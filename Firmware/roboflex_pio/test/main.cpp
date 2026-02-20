#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>

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
static constexpr uint8_t kPwmChannelMap[kJointCount] = {0, 1, 2, 3, 4};
static constexpr float kJointZeroOffsetDeg[kJointCount] = {
  -90.0f,  // base joint offset compensation
  0.0f,
  0.0f,
  0.0f,
  0.0f
};

// Motion smoothing on firmware side (lower step / higher interval = slower motion)
const float stepSizeDeg = 0.5f;  // degrees per cycle
const unsigned long updateIntervalMs = 20;

unsigned long lastUpdateTime = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long lastCommandMs = 0;
unsigned long receivedCommandCount = 0;
unsigned long lastSizeWarnMs = 0;

float currentAngleDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float targetAngleDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float radToServoDeg(float rad_value, float zero_offset_deg) {
  float servo_deg = mapFloat(clampRad(rad_value), -1.57f, 1.57f, 0.0f, 180.0f);
  return clampDeg(servo_deg + zero_offset_deg);
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
    targetAngleDeg[i] = radToServoDeg(rad, kJointZeroOffsetDeg[i]);
  }

  lastCommandMs = millis();
  receivedCommandCount++;
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

  // Startup pose: kinematic zero + per-joint offsets.
  for (size_t i = 0; i < kJointCount; ++i) {
    currentAngleDeg[i] = radToServoDeg(0.0f, kJointZeroOffsetDeg[i]);
    targetAngleDeg[i] = currentAngleDeg[i];
    writeServoByIndex(i, currentAngleDeg[i]);
  }
  lastCommandMs = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdateTime >= updateIntervalMs) {
    lastUpdateTime = now;

    for (size_t i = 0; i < kJointCount; ++i) {
      if (currentAngleDeg[i] < targetAngleDeg[i]) {
        currentAngleDeg[i] += stepSizeDeg;
        if (currentAngleDeg[i] > targetAngleDeg[i]) currentAngleDeg[i] = targetAngleDeg[i];
      } else if (currentAngleDeg[i] > targetAngleDeg[i]) {
        currentAngleDeg[i] -= stepSizeDeg;
        if (currentAngleDeg[i] < targetAngleDeg[i]) currentAngleDeg[i] = targetAngleDeg[i];
      }
      writeServoByIndex(i, currentAngleDeg[i]);
    }
  }

  if (now - lastStatusPrintMs >= 2000) {
    lastStatusPrintMs = now;
    Serial.print("[STATUS] rx_count=");
    Serial.print((unsigned long)receivedCommandCount);
    Serial.print(" last_rx_ms_ago=");
    Serial.print((unsigned long)(now - lastCommandMs));
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
