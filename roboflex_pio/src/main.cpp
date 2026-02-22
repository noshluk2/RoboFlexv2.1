#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50

#define LED_PIN 2

// Network config can be overridden from platformio.ini via build_flags:
// -D ROBOFLEX_WIFI_SSID=\"MySSID\"
// -D ROBOFLEX_WIFI_PASSWORD=\"MyPassword\"
// -D ROBOFLEX_UDP_LISTEN_PORT=9999
#ifndef ROBOFLEX_WIFI_SSID
#define ROBOFLEX_WIFI_SSID "Connect"
#endif

#ifndef ROBOFLEX_WIFI_PASSWORD
#define ROBOFLEX_WIFI_PASSWORD "123456789"
#endif

#ifndef ROBOFLEX_UDP_LISTEN_PORT
#define ROBOFLEX_UDP_LISTEN_PORT 9999
#endif

#ifndef ROBOFLEX_PCA9685_ADDR
#define ROBOFLEX_PCA9685_ADDR 0x40
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(ROBOFLEX_PCA9685_ADDR);
Preferences preferences;
WiFiUDP command_udp;

static constexpr size_t kJointCount = 5;
static constexpr size_t kMinCommandJoints = 4;
static constexpr size_t kUdpMaxPacketSize = 192;
static constexpr unsigned long kWifiConnectTimeoutMs = 20000;
// Physical wiring on RoboFlex uses PCA9685 channels 1..5 for:
// joint_1, joint_2, joint_3, joint_4, joint_gripper respectively.
static constexpr uint8_t kPwmChannelMap[kJointCount] = {1, 2, 3, 4, 5};
static constexpr float kStartupPoseDeg[kJointCount] = {
  0.0f,  // joint_1
  0.0f,  // joint_2
  90.0f, // joint_3
  90.0f, // joint_4
  90.0f  // joint_gripper
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
char udpPacketBuffer[kUdpMaxPacketSize + 1];

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

bool probePca9685(uint8_t address, uint8_t retries) {
  for (uint8_t attempt = 1; attempt <= retries; ++attempt) {
    Wire.beginTransmission(address);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      return true;
    }

    Serial.print("[WARN] PCA9685 probe failed addr=0x");
    Serial.print(address, HEX);
    Serial.print(" code=");
    Serial.print((int)err);
    Serial.print(" attempt=");
    Serial.print((int)attempt);
    Serial.print("/");
    Serial.println((int)retries);
    delay(120);
  }
  return false;
}

bool connectToWifi(const char *ssid, const char *password) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < kWifiConnectTimeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  return WiFi.status() == WL_CONNECTED;
}

bool parseUdpCommandPacket(char *packet, size_t bytes_read, float *commands_rad, size_t &command_count) {
  if (bytes_read == 0) {
    return false;
  }

  packet[bytes_read] = '\0';
  for (size_t i = 0; i < bytes_read; ++i) {
    if (packet[i] == ',' || packet[i] == ';') {
      packet[i] = ' ';
    }
  }

  char *save_ptr = nullptr;
  char *token = strtok_r(packet, " \t\r\n", &save_ptr);
  if (token == nullptr) {
    return false;
  }

  if (strcmp(token, "CMD") == 0 || strcmp(token, "cmd") == 0 ||
      strcmp(token, "MOTOR") == 0 || strcmp(token, "motor") == 0) {
    token = strtok_r(nullptr, " \t\r\n", &save_ptr);
  }

  command_count = 0;
  while (token != nullptr && command_count < kJointCount) {
    commands_rad[command_count] = strtof(token, nullptr);
    ++command_count;
    token = strtok_r(nullptr, " \t\r\n", &save_ptr);
  }

  return command_count >= kMinCommandJoints;
}

void applyCommandRadians(const float *commands_rad, size_t command_count) {
  if (command_count < kMinCommandJoints) {
    return;
  }

  if (command_count != kJointCount && millis() - lastSizeWarnMs > 2000) {
    lastSizeWarnMs = millis();
    Serial.print("[WARN] UDP command count=");
    Serial.print((int)command_count);
    Serial.print(" expected=");
    Serial.println((int)kJointCount);
  }

  for (size_t i = 0; i < kJointCount; ++i) {
    float rad = 0.0f;
    if (i < command_count) {
      rad = commands_rad[i];
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

void pollUdpCommands() {
  int packet_size = command_udp.parsePacket();
  while (packet_size > 0) {
    const int bytes_read = command_udp.read(udpPacketBuffer, kUdpMaxPacketSize);
    if (bytes_read > 0) {
      float commands_rad[kJointCount] = {0.0f};
      size_t command_count = 0;
      if (parseUdpCommandPacket(udpPacketBuffer, (size_t)bytes_read, commands_rad, command_count)) {
        applyCommandRadians(commands_rad, command_count);
      } else if (millis() - lastSizeWarnMs > 2000) {
        lastSizeWarnMs = millis();
        Serial.print("[WARN] Invalid UDP command packet: '");
        Serial.print(udpPacketBuffer);
        Serial.println("'");
      }
    }

    packet_size = command_udp.parsePacket();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Starting RoboFlex UDP firmware node");

  static char wifi_ssid[] = ROBOFLEX_WIFI_SSID;
  static char wifi_password[] = ROBOFLEX_WIFI_PASSWORD;

  if (!connectToWifi(wifi_ssid, wifi_password)) {
    Serial.println("[ERROR] WiFi connection failed");
    error_loop();
  }

  Serial.print("WiFi connected. Local IP: ");
  Serial.println(WiFi.localIP());

  if (!command_udp.begin(ROBOFLEX_UDP_LISTEN_PORT)) {
    Serial.println("[ERROR] Failed to start UDP listener");
    error_loop();
  }

  Serial.print("Listening for UDP motor commands on port ");
  Serial.println((int)ROBOFLEX_UDP_LISTEN_PORT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!probePca9685(ROBOFLEX_PCA9685_ADDR, 15)) {
    Serial.println("[ERROR] PCA9685 not detected on I2C bus.");
    Serial.println("[ERROR] Check SDA/SCL wiring, 5V power, GND common, OE pin, and address jumpers.");
    error_loop();
  }

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  resetJointSoftLimitsToDefaults();
  const bool loaded_from_nvs = loadJointSoftLimitsFromNvs();
  if (!loaded_from_nvs) {
    Serial.println("[INFO] Running with compiled default joint soft limits");
  }

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

  pollUdpCommands();

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
    Serial.print(" wifi=");
    Serial.print((WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected");
    Serial.print(" speed_mode=");
    Serial.print(startupSlowModeActive ? "startup_slow" : "normal");
    Serial.print(" target_deg=[");
    for (size_t i = 0; i < kJointCount; ++i) {
      Serial.print(targetAngleDeg[i], 1);
      if (i + 1 < kJointCount) Serial.print(", ");
    }
    Serial.println("]");

    Serial.print("[STATUS] current_deg=[");
    for (size_t i = 0; i < kJointCount; ++i) {
      Serial.print(currentAngleDeg[i], 1);
      if (i + 1 < kJointCount) Serial.print(", ");
    }
    Serial.println("]");
  }
}
