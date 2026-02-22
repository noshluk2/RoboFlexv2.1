#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50

#define LED_PIN 2
#define COMMAND_TOPIC "/motor_command"

// Network config can be overridden from platformio.ini via build_flags:
// -D ROBOFLEX_WIFI_SSID=\"MySSID\"
// -D ROBOFLEX_WIFI_PASSWORD=\"MyPassword\"
// -D ROBOFLEX_AGENT_IP=\"192.168.1.10\"
// -D ROBOFLEX_AGENT_PORT=8888
#ifndef ROBOFLEX_WIFI_SSID
#define ROBOFLEX_WIFI_SSID "Connect"
#endif

#ifndef ROBOFLEX_WIFI_PASSWORD
#define ROBOFLEX_WIFI_PASSWORD "123456789"
#endif

#ifndef ROBOFLEX_AGENT_IP
#define ROBOFLEX_AGENT_IP "10.54.47.220"
#endif

#ifndef ROBOFLEX_AGENT_PORT
#define ROBOFLEX_AGENT_PORT 8888
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;

rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)                                 \
  {                                                 \
    rcl_ret_t temp_rc = fn;                         \
    if ((temp_rc != RCL_RET_OK)) {                  \
      error_loop();                                 \
    }                                               \
  }

#define RCSOFTCHECK(fn)                             \
  {                                                 \
    rcl_ret_t temp_rc = fn;                         \
    if ((temp_rc != RCL_RET_OK)) {                  \
    }                                               \
  }

static constexpr size_t kJointCount = 5;
static constexpr size_t kMinCommandJoints = 4;
// Physical wiring on RoboFlex uses PCA9685 channels 1..5 for:
// joint_1, joint_2, joint_3, joint_4, joint_gripper respectively.
static constexpr uint8_t kPwmChannelMap[kJointCount] = {1, 2, 3, 4, 5};
static constexpr float kStartupPoseDeg[kJointCount] = {
  0.0f,  // joint_1
  0.0f,  // joint_2
  90.0f, // joint_3
  90.0f, // joint_gripper_body
  90.0f  // joint_gripper_left_part
};
static constexpr float kJointZeroOffsetDeg[kJointCount] = {
  -90.0f,  // base joint offset compensation
  0.0f,
  0.0f,
  0.0f,
  0.0f
};
static constexpr float kJointDefaultMinDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static constexpr float kJointDefaultMaxDeg[kJointCount] = {120.0f, 120.0f, 80.0f, 120.0f, 120.0f};
// Reverse only joint_2 command direction to match physical motor orientation.
static constexpr bool kJointInvertCommand[kJointCount] = {false, true, false, false, false};
static constexpr uint32_t kJointLimitsPresetVersion = 2;
static constexpr const char *kJointLimitsNamespace = "motor_limits";

// Motion smoothing on firmware side (lower step / higher interval = slower motion)
const float normalStepSizeDeg = 0.5f;     // degrees per cycle (normal operation)
const float startupStepSizeDeg = 0.15f;   // safer speed while settling initial pose
const float targetReachedEpsilonDeg = 0.2f;
const unsigned long updateIntervalMs = 20;

unsigned long lastUpdateTime = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long lastCommandMs = 0;
unsigned long receivedCommandCount = 0;
unsigned long lastSizeWarnMs = 0;
bool hasReceivedExternalCommand = false;
bool startupSlowModeActive = true;

float currentAngleDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float targetAngleDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float jointSafeMinDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float jointSafeMaxDeg[kJointCount] = {180.0f, 180.0f, 180.0f, 180.0f, 180.0f};
double msg_buffer[kJointCount];

float clampRad(float rad_value) {
  if (rad_value > 1.57f) return 1.57f;
  if (rad_value < -1.57f) return -1.57f;
  return rad_value;
}

float clampDeg(float deg_value) {
  if (deg_value > 180.0f) return 180.0f;
  if (deg_value < 0.0f) return 0.0f;
  return deg_value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float radToServoDeg(float rad_value, float zero_offset_deg) {
  float servo_deg = mapFloat(clampRad(rad_value), -1.57f, 1.57f, 0.0f, 180.0f);
  return clampDeg(servo_deg + zero_offset_deg);
}

void resetJointSoftLimitsToDefaults() {
  for (size_t i = 0; i < kJointCount; ++i) {
    jointSafeMinDeg[i] = clampDeg(kJointDefaultMinDeg[i]);
    jointSafeMaxDeg[i] = clampDeg(kJointDefaultMaxDeg[i]);
    if (jointSafeMinDeg[i] > jointSafeMaxDeg[i]) {
      const float tmp = jointSafeMinDeg[i];
      jointSafeMinDeg[i] = jointSafeMaxDeg[i];
      jointSafeMaxDeg[i] = tmp;
    }
  }
}

bool loadJointSoftLimitsFromNvs() {
  if (!preferences.begin(kJointLimitsNamespace, true)) {
    Serial.println("[WARN] NVS open failed for limit load; using defaults");
    return false;
  }

  const bool valid = preferences.getBool("valid", false);
  const uint32_t stored_count = preferences.getUInt("count", 0);
  const uint32_t stored_preset = preferences.getUInt("preset", 0);

  if (!valid || stored_count < kJointCount || stored_preset != kJointLimitsPresetVersion) {
    preferences.end();
    Serial.println("[WARN] No compatible NVS limits; using defaults");
    return false;
  }

  for (size_t i = 0; i < kJointCount; ++i) {
    char key[16];

    snprintf(key, sizeof(key), "min%u", (unsigned)i);
    jointSafeMinDeg[i] = clampDeg(preferences.getFloat(key, kJointDefaultMinDeg[i]));

    snprintf(key, sizeof(key), "max%u", (unsigned)i);
    jointSafeMaxDeg[i] = clampDeg(preferences.getFloat(key, kJointDefaultMaxDeg[i]));

    if (jointSafeMinDeg[i] > jointSafeMaxDeg[i]) {
      const float tmp = jointSafeMinDeg[i];
      jointSafeMinDeg[i] = jointSafeMaxDeg[i];
      jointSafeMaxDeg[i] = tmp;
    }
  }

  preferences.end();
  Serial.println("[OK] Loaded joint soft limits from NVS");
  return true;
}

bool saveJointSoftLimitsToNvs() {
  if (!preferences.begin(kJointLimitsNamespace, false)) {
    Serial.println("[ERR] NVS open failed for limit save");
    return false;
  }

  preferences.putBool("valid", true);
  preferences.putUInt("count", (uint32_t)kJointCount);
  preferences.putUInt("preset", kJointLimitsPresetVersion);

  for (size_t i = 0; i < kJointCount; ++i) {
    char key[16];

    snprintf(key, sizeof(key), "min%u", (unsigned)i);
    preferences.putFloat(key, jointSafeMinDeg[i]);

    snprintf(key, sizeof(key), "max%u", (unsigned)i);
    preferences.putFloat(key, jointSafeMaxDeg[i]);
  }

  preferences.end();
  Serial.println("[OK] Saved joint soft limits to NVS");
  return true;
}

float clampToJointSoftLimits(size_t joint_idx, float target_deg) {
  if (joint_idx >= kJointCount) return clampDeg(target_deg);
  return clampFloat(clampDeg(target_deg), jointSafeMinDeg[joint_idx], jointSafeMaxDeg[joint_idx]);
}

void writeServoByIndex(size_t idx, float angle_deg) {
  int pulse = (int)mapFloat(angle_deg, 0.0f, 180.0f, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(kPwmChannelMap[idx], 0, pulse);
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *in_msg =
      (const std_msgs__msg__Float64MultiArray *)msgin;

  if (in_msg->data.size < kMinCommandJoints) {
    Serial.print("[WARN] /motor_command size too small: ");
    Serial.println((int)in_msg->data.size);
    return;
  }

  if (in_msg->data.size != kJointCount && millis() - lastSizeWarnMs > 2000) {
    lastSizeWarnMs = millis();
    Serial.print("[WARN] /motor_command size=");
    Serial.print((int)in_msg->data.size);
    Serial.print(" expected=");
    Serial.println((int)kJointCount);
  }

  for (size_t i = 0; i < kJointCount; ++i) {
    float rad = 0.0f;
    if (i < in_msg->data.size) {
      rad = (float)in_msg->data.data[i];
    }
    float mapped_deg = radToServoDeg(rad, kJointZeroOffsetDeg[i]);
    if (kJointInvertCommand[i]) {
      mapped_deg = (jointSafeMinDeg[i] + jointSafeMaxDeg[i]) - mapped_deg;
    }
    targetAngleDeg[i] = clampToJointSoftLimits(i, mapped_deg);
  }

  lastCommandMs = millis();
  receivedCommandCount++;
  hasReceivedExternalCommand = true;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting Micro-ROS ESP32 Node");

  static char wifi_ssid[] = ROBOFLEX_WIFI_SSID;
  static char wifi_password[] = ROBOFLEX_WIFI_PASSWORD;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  IPAddress agent_ip;
  if (!agent_ip.fromString(ROBOFLEX_AGENT_IP)) {
    Serial.print("[ERROR] Invalid ROBOFLEX_AGENT_IP: ");
    Serial.println(ROBOFLEX_AGENT_IP);
    error_loop();
  }

  set_microros_wifi_transports(
      wifi_ssid,
      wifi_password,
      agent_ip,
      ROBOFLEX_AGENT_PORT);

  Serial.print("micro-ROS WiFi SSID: ");
  Serial.println(wifi_ssid);
  Serial.print("micro-ROS agent: ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.println((int)ROBOFLEX_AGENT_PORT);

  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  resetJointSoftLimitsToDefaults();
  const bool loaded_from_nvs = loadJointSoftLimitsFromNvs();
  if (!loaded_from_nvs) {
    Serial.println("[INFO] Running with compiled default joint soft limits");
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "hardware_driver_node", "", &support));

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = 1;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  RCCHECK(rclc_subscription_init(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      COMMAND_TOPIC,
      &qos_profile));
  Serial.print("Subscribed to: ");
  Serial.println(COMMAND_TOPIC);

  std_msgs__msg__Float64MultiArray__init(&msg);
  msg.data.data = msg_buffer;
  msg.data.size = kJointCount;
  msg.data.capacity = kJointCount;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Startup pose in servo degrees (clamped by per-joint safety limits).
  for (size_t i = 0; i < kJointCount; ++i) {
    currentAngleDeg[i] = clampToJointSoftLimits(i, kStartupPoseDeg[i]);
    targetAngleDeg[i] = currentAngleDeg[i];
    writeServoByIndex(i, currentAngleDeg[i]);
  }
  lastCommandMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdateTime >= updateIntervalMs) {
    lastUpdateTime = now;
    const float activeStepDeg = startupSlowModeActive ? startupStepSizeDeg : normalStepSizeDeg;
    bool allTargetsReached = true;

    for (size_t i = 0; i < kJointCount; ++i) {
      if (currentAngleDeg[i] < targetAngleDeg[i]) {
        currentAngleDeg[i] += activeStepDeg;
        if (currentAngleDeg[i] > targetAngleDeg[i]) currentAngleDeg[i] = targetAngleDeg[i];
      } else if (currentAngleDeg[i] > targetAngleDeg[i]) {
        currentAngleDeg[i] -= activeStepDeg;
        if (currentAngleDeg[i] < targetAngleDeg[i]) currentAngleDeg[i] = targetAngleDeg[i];
      }

      float remainingDeg = currentAngleDeg[i] - targetAngleDeg[i];
      if (remainingDeg < 0.0f) remainingDeg = -remainingDeg;
      if (remainingDeg > targetReachedEpsilonDeg) allTargetsReached = false;
      writeServoByIndex(i, currentAngleDeg[i]);
    }

    if (startupSlowModeActive && hasReceivedExternalCommand && allTargetsReached) {
      startupSlowModeActive = false;
      Serial.println("[INFO] Startup pose settled; switching to normal motion speed");
    }
  }

  if (now - lastStatusPrintMs >= 2000) {
    lastStatusPrintMs = now;
    Serial.print("[STATUS] rx_count=");
    Serial.print((unsigned long)receivedCommandCount);
    Serial.print(" last_rx_ms_ago=");
    Serial.print((unsigned long)(now - lastCommandMs));
    Serial.print(" speed_mode=");
    Serial.print(startupSlowModeActive ? "startup_slow" : "normal");
    Serial.print(" target_deg=[");
    for (size_t i = 0; i < kJointCount; ++i) {
      Serial.print(targetAngleDeg[i], 1);
      if (i + 1 < kJointCount) Serial.print(", ");
    }
    Serial.println("]");
  }

  // Keep executor responsive without stalling servo update loop.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
}
