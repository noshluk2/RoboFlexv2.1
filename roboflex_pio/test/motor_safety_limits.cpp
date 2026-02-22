#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#include <math.h>
#include <string.h>

// ------------------------------
// Hardware configuration
// ------------------------------
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;
static constexpr uint8_t STATUS_LED_PIN = 2;

static constexpr uint16_t SERVO_MIN_PULSE = 150;
static constexpr uint16_t SERVO_MAX_PULSE = 600;
static constexpr uint8_t SERVO_FREQ_HZ = 50;

// Set this to 5 if your gripper channel is not connected in this test phase.
static constexpr size_t kMotorCount = 6;
static constexpr uint8_t kPwmChannelMap[kMotorCount] = {0, 1, 2, 3, 4, 5};
static constexpr const char *kJointLabels[kMotorCount] = {
  "joint0", "joint1", "joint2", "joint3", "joint4", "gripper"
};

static constexpr float kNeutralStartDeg = 90.0f;
static constexpr float kAbsoluteMinDeg = 0.0f;
static constexpr float kAbsoluteMaxDeg = 180.0f;

// Calibrated defaults from latest bench test.
// joint 0 is set conservatively to 120 until explicitly re-calibrated.
static constexpr float kDefaultSafeMinDeg[kMotorCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static constexpr float kDefaultSafeMaxDeg[kMotorCount] = {120.0f, 120.0f, 80.0f, 120.0f, 120.0f, 120.0f};
static constexpr float kStartupPoseDeg[kMotorCount] = {90.0f, 0.0f, 90.0f, 90.0f, 90.0f, 90.0f};

static constexpr uint32_t kLimitsPresetVersion = 2;
static constexpr float kNormalSmoothingStepDeg = 1.0f;
static constexpr float kStartupSmoothingStepDeg = 0.25f;
static constexpr unsigned long kSmoothingIntervalMs = 20;
// 5 in-range joint goal sets (normalized 0..1 of [safe_min, safe_max]).
static constexpr float kGoalSetFractions[][kMotorCount] = {
  {0.20f, 0.05f, 0.25f, 0.20f, 0.20f, 0.00f},
  {0.40f, 0.25f, 0.45f, 0.35f, 0.40f, 1.00f},
  {0.60f, 0.50f, 0.65f, 0.55f, 0.60f, 0.30f},
  {0.80f, 0.75f, 0.40f, 0.75f, 0.80f, 1.00f},
  {0.50f, 0.35f, 0.70f, 0.45f, 0.55f, 0.00f}
};
static constexpr size_t kGoalSetCount = sizeof(kGoalSetFractions) / sizeof(kGoalSetFractions[0]);
static constexpr unsigned long kGoalSetGapMs = 3000;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Preferences preferences;

struct MotorState {
  float current_deg;
  float target_deg;
  float safe_min_deg;
  float safe_max_deg;
  bool has_captured_min;
  bool has_captured_max;
};

MotorState motors[kMotorCount];

char serialLine[128];
size_t serialLineLen = 0;

unsigned long lastSmoothingMs = 0;
bool startupPoseInProgress = true;
bool jointGoalSetTestActive = false;
size_t jointGoalSetIndex = 0;
unsigned long jointGoalSetLastStepMs = 0;

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int angleDegToPulse(float angle_deg) {
  const float clamped = clampFloat(angle_deg, kAbsoluteMinDeg, kAbsoluteMaxDeg);
  const float normalized = (clamped - kAbsoluteMinDeg) / (kAbsoluteMaxDeg - kAbsoluteMinDeg);
  return (int)roundf(SERVO_MIN_PULSE + normalized * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

void writeMotor(size_t joint, float angle_deg) {
  if (joint >= kMotorCount) return;
  pwm.setPWM(kPwmChannelMap[joint], 0, angleDegToPulse(angle_deg));
}

void printHelp() {
  Serial.println();
  Serial.println("=== RoboFlex Motor Safety Limit Tool ===");
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  status");
  Serial.println("  m <joint> <deg>          # move with soft limits");
  Serial.println("  raw <joint> <deg>        # move ignoring soft limits (0..180 only)");
  Serial.println("  step <joint> <delta_deg> # increment/decrement target");
  Serial.println("  all <deg>                # set same target for all joints");
  Serial.println("  setmin <joint>           # capture current angle as safe min");
  Serial.println("  setmax <joint>           # capture current angle as safe max");
  Serial.println("  limits                # show safe limit table");
  Serial.println("  save                  # persist limits to NVS");
  Serial.println("  load                  # load limits from NVS");
  Serial.println("  reset_limits          # reset to calibrated defaults");
  Serial.println("  export                # print C++ arrays for firmware");
  Serial.println("  center                   # move all joints to 90 deg");
  Serial.println("  test_sets                # run 5 in-range joint goal sets (3s gap)");
  Serial.println("  test_array               # alias for test_sets");
  Serial.println("  stop_test                # stop running goal-set test");
  Serial.println("  <j1..j5 or j0..j5>       # direct joint goals (space-separated)");
  Serial.println();
  Serial.println("Joint indexes: 0..5 (joint 5 = gripper, mapped to PCA9685 channels 0..5)");
}

void printLimits() {
  Serial.println("joint,ch,name,current,target,safe_min,safe_max,captured_min,captured_max");
  for (size_t i = 0; i < kMotorCount; ++i) {
    Serial.print(i);
    Serial.print(",");
    Serial.print(kPwmChannelMap[i]);
    Serial.print(",");
    Serial.print(kJointLabels[i]);
    Serial.print(",");
    Serial.print(motors[i].current_deg, 1);
    Serial.print(",");
    Serial.print(motors[i].target_deg, 1);
    Serial.print(",");
    Serial.print(motors[i].safe_min_deg, 1);
    Serial.print(",");
    Serial.print(motors[i].safe_max_deg, 1);
    Serial.print(",");
    Serial.print(motors[i].has_captured_min ? "yes" : "no");
    Serial.print(",");
    Serial.println(motors[i].has_captured_max ? "yes" : "no");
  }
}

void exportLimitsForFirmware() {
  Serial.println("Copy this into your firmware when validated:");

  Serial.print("static constexpr float kJointMinDeg[");
  Serial.print(kMotorCount);
  Serial.print("] = {");
  for (size_t i = 0; i < kMotorCount; ++i) {
    Serial.print(motors[i].safe_min_deg, 1);
    if (i + 1 < kMotorCount) Serial.print(", ");
  }
  Serial.println("};");

  Serial.print("static constexpr float kJointMaxDeg[");
  Serial.print(kMotorCount);
  Serial.print("] = {");
  for (size_t i = 0; i < kMotorCount; ++i) {
    Serial.print(motors[i].safe_max_deg, 1);
    if (i + 1 < kMotorCount) Serial.print(", ");
  }
  Serial.println("};");
}

void resetSoftLimits() {
  for (size_t i = 0; i < kMotorCount; ++i) {
    motors[i].safe_min_deg = clampFloat(kDefaultSafeMinDeg[i], kAbsoluteMinDeg, kAbsoluteMaxDeg);
    motors[i].safe_max_deg = clampFloat(kDefaultSafeMaxDeg[i], kAbsoluteMinDeg, kAbsoluteMaxDeg);
    if (motors[i].safe_min_deg > motors[i].safe_max_deg) {
      const float tmp = motors[i].safe_min_deg;
      motors[i].safe_min_deg = motors[i].safe_max_deg;
      motors[i].safe_max_deg = tmp;
    }
    motors[i].has_captured_min = true;
    motors[i].has_captured_max = true;
  }
}

void saveLimitsToNvs() {
  if (!preferences.begin("motor_limits", false)) {
    Serial.println("[ERR] Failed to open NVS for writing");
    return;
  }

  preferences.putUInt("count", (uint32_t)kMotorCount);
  preferences.putUInt("preset", kLimitsPresetVersion);
  preferences.putBool("valid", true);

  for (size_t i = 0; i < kMotorCount; ++i) {
    char key[16];

    snprintf(key, sizeof(key), "min%u", (unsigned)i);
    preferences.putFloat(key, motors[i].safe_min_deg);

    snprintf(key, sizeof(key), "max%u", (unsigned)i);
    preferences.putFloat(key, motors[i].safe_max_deg);

    snprintf(key, sizeof(key), "cmin%u", (unsigned)i);
    preferences.putBool(key, motors[i].has_captured_min);

    snprintf(key, sizeof(key), "cmax%u", (unsigned)i);
    preferences.putBool(key, motors[i].has_captured_max);
  }

  preferences.end();
  Serial.println("[OK] Limits saved to NVS");
}

void loadLimitsFromNvs() {
  if (!preferences.begin("motor_limits", true)) {
    Serial.println("[ERR] Failed to open NVS for reading");
    return;
  }

  const bool valid = preferences.getBool("valid", false);
  const uint32_t stored_count = preferences.getUInt("count", 0);
  const uint32_t stored_preset = preferences.getUInt("preset", 0);

  if (!valid || stored_count != kMotorCount || stored_preset != kLimitsPresetVersion) {
    preferences.end();
    Serial.println("[WARN] No compatible saved limits found; using calibrated defaults");
    return;
  }

  for (size_t i = 0; i < kMotorCount; ++i) {
    char key[16];

    snprintf(key, sizeof(key), "min%u", (unsigned)i);
    motors[i].safe_min_deg = clampFloat(preferences.getFloat(key, kAbsoluteMinDeg), kAbsoluteMinDeg, kAbsoluteMaxDeg);

    snprintf(key, sizeof(key), "max%u", (unsigned)i);
    motors[i].safe_max_deg = clampFloat(preferences.getFloat(key, kAbsoluteMaxDeg), kAbsoluteMinDeg, kAbsoluteMaxDeg);

    snprintf(key, sizeof(key), "cmin%u", (unsigned)i);
    motors[i].has_captured_min = preferences.getBool(key, false);

    snprintf(key, sizeof(key), "cmax%u", (unsigned)i);
    motors[i].has_captured_max = preferences.getBool(key, false);

    if (motors[i].safe_min_deg > motors[i].safe_max_deg) {
      const float tmp = motors[i].safe_min_deg;
      motors[i].safe_min_deg = motors[i].safe_max_deg;
      motors[i].safe_max_deg = tmp;
    }
  }

  preferences.end();
  Serial.println("[OK] Loaded saved limits from NVS");
}

bool parseIntToken(const char *token, int &out) {
  if (token == nullptr) return false;
  char *end_ptr = nullptr;
  const long value = strtol(token, &end_ptr, 10);
  if (end_ptr == token || *end_ptr != '\0') return false;
  out = (int)value;
  return true;
}

bool parseFloatToken(const char *token, float &out) {
  if (token == nullptr) return false;
  char *end_ptr = nullptr;
  const float value = strtof(token, &end_ptr);
  if (end_ptr == token || *end_ptr != '\0') return false;
  out = value;
  return true;
}

bool validateJointIndex(int joint) {
  if (joint < 0 || joint >= (int)kMotorCount) {
    Serial.print("[ERR] joint out of range: ");
    Serial.println(joint);
    return false;
  }
  return true;
}

void setMotorTargetDeg(size_t joint, float requested_deg, bool apply_soft_limits, bool print_ack = true) {
  float target = clampFloat(requested_deg, kAbsoluteMinDeg, kAbsoluteMaxDeg);

  if (apply_soft_limits) {
    target = clampFloat(target, motors[joint].safe_min_deg, motors[joint].safe_max_deg);
  }

  motors[joint].target_deg = target;
  if (!print_ack) return;

  Serial.print("[OK] joint ");
  Serial.print(joint);
  Serial.print(" target=");
  Serial.print(target, 1);
  Serial.print(" (requested ");
  Serial.print(requested_deg, 1);
  Serial.println(")");
}

bool tryParseJointVectorInput(const char *line, float *out_values, size_t &out_count) {
  if (line == nullptr || out_values == nullptr) return false;

  char copy[sizeof(serialLine)];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  out_count = 0;
  char *token = strtok(copy, " \t");
  if (token == nullptr) return false;

  while (token != nullptr) {
    if (out_count >= kMotorCount) return false;
    if (!parseFloatToken(token, out_values[out_count])) return false;
    out_count++;
    token = strtok(nullptr, " \t");
  }

  // Accept either 5 values (joints 1..5, keep joint 0 target) or 6 values (0..5).
  return (out_count == (kMotorCount - 1)) || (out_count == kMotorCount);
}

void applyJointVectorTargets(const float *values, size_t count) {
  if (values == nullptr || count == 0) return;

  Serial.print("[OK] vector input = [");
  for (size_t i = 0; i < count; ++i) {
    Serial.print(values[i], 1);
    if (i + 1 < count) Serial.print(", ");
  }
  Serial.print("]");

  if (count == (kMotorCount - 1)) {
    Serial.print(" -> applying joints 1..");
    Serial.print((int)(kMotorCount - 1));
    Serial.print(", keeping joint 0 (");
    Serial.print(kJointLabels[0]);
    Serial.print(") at ");
    Serial.print(motors[0].target_deg, 1);
    Serial.println();
    for (size_t i = 0; i < count; ++i) {
      setMotorTargetDeg(i + 1, values[i], true, false);
    }
    return;
  }

  Serial.print(" -> applying all joints 0..");
  Serial.print((int)(kMotorCount - 1));
  Serial.println();

  for (size_t i = 0; i < count; ++i) {
    setMotorTargetDeg(i, values[i], true, false);
  }
}

float getJointGoalSetTargetDeg(size_t joint, size_t set_index) {
  if (joint >= kMotorCount) return kAbsoluteMinDeg;
  if (set_index >= kGoalSetCount) set_index = kGoalSetCount - 1;

  const float safe_min = motors[joint].safe_min_deg;
  const float safe_max = motors[joint].safe_max_deg;
  const float range = safe_max - safe_min;
  const float fraction = clampFloat(kGoalSetFractions[set_index][joint], 0.0f, 1.0f);

  if (range <= 0.0f) return safe_min;
  return safe_min + (range * fraction);
}

void stopJointGoalSetTest(const char *reason) {
  if (!jointGoalSetTestActive) return;
  jointGoalSetTestActive = false;
  Serial.print("[OK] joint goal-set test stopped");
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println(".");
}

void runJointGoalSetTestStep(unsigned long now) {
  if (!jointGoalSetTestActive) return;
  if (jointGoalSetLastStepMs != 0 && now - jointGoalSetLastStepMs < kGoalSetGapMs) return;

  if (jointGoalSetIndex >= kGoalSetCount) {
    jointGoalSetTestActive = false;
    Serial.println("[OK] joint goal-set test completed.");
    return;
  }

  const size_t set_index = jointGoalSetIndex;
  Serial.print("[TEST] goal set ");
  Serial.print(set_index + 1);
  Serial.print("/");
  Serial.println(kGoalSetCount);

  for (size_t joint = 0; joint < kMotorCount; ++joint) {
    const float target = getJointGoalSetTargetDeg(joint, set_index);
    setMotorTargetDeg(joint, target, true);
  }

  jointGoalSetLastStepMs = now;
  jointGoalSetIndex++;

  if (jointGoalSetIndex >= kGoalSetCount) {
    jointGoalSetTestActive = false;
    Serial.println("[OK] joint goal-set test completed.");
  }
}

void startJointGoalSetTest(unsigned long now) {
  jointGoalSetTestActive = true;
  jointGoalSetIndex = 0;
  jointGoalSetLastStepMs = 0;

  Serial.print("[OK] joint goal-set test started: ");
  Serial.print(kGoalSetCount);
  Serial.print(" sets, ");
  Serial.print(kGoalSetGapMs);
  Serial.println(" ms gap.");

  runJointGoalSetTestStep(now);
}

void processCommand(char *line) {
  char raw_line[sizeof(serialLine)];
  strncpy(raw_line, line, sizeof(raw_line) - 1);
  raw_line[sizeof(raw_line) - 1] = '\0';

  float vector_values[kMotorCount];
  size_t vector_count = 0;
  if (tryParseJointVectorInput(raw_line, vector_values, vector_count)) {
    stopJointGoalSetTest("manual command");
    applyJointVectorTargets(vector_values, vector_count);
    return;
  }

  char *cmd = strtok(line, " \t");
  if (cmd == nullptr) return;

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "status") == 0 || strcmp(cmd, "limits") == 0) {
    printLimits();
    return;
  }

  if (strcmp(cmd, "test_sets") == 0 || strcmp(cmd, "test_array") == 0 || strcmp(cmd, "test") == 0) {
    startJointGoalSetTest(millis());
    return;
  }

  if (strcmp(cmd, "stop_test") == 0) {
    stopJointGoalSetTest("user");
    return;
  }

  if (strcmp(cmd, "center") == 0) {
    stopJointGoalSetTest("manual command");
    for (size_t i = 0; i < kMotorCount; ++i) {
      motors[i].target_deg = kNeutralStartDeg;
    }
    Serial.println("[OK] all joints target=90");
    return;
  }

  if (strcmp(cmd, "all") == 0) {
    stopJointGoalSetTest("manual command");
    float deg = 0.0f;
    if (!parseFloatToken(strtok(nullptr, " \t"), deg)) {
      Serial.println("[ERR] usage: all <deg>");
      return;
    }
    for (size_t i = 0; i < kMotorCount; ++i) {
      setMotorTargetDeg(i, deg, true);
    }
    return;
  }

  if (strcmp(cmd, "m") == 0 || strcmp(cmd, "move") == 0 || strcmp(cmd, "raw") == 0) {
    stopJointGoalSetTest("manual command");
    int joint = -1;
    float deg = 0.0f;
    if (!parseIntToken(strtok(nullptr, " \t"), joint) || !parseFloatToken(strtok(nullptr, " \t"), deg)) {
      Serial.println("[ERR] usage: m <joint> <deg>");
      return;
    }
    if (!validateJointIndex(joint)) return;
    const bool apply_soft_limits = strcmp(cmd, "raw") != 0;
    setMotorTargetDeg((size_t)joint, deg, apply_soft_limits);
    return;
  }

  if (strcmp(cmd, "step") == 0) {
    stopJointGoalSetTest("manual command");
    int joint = -1;
    float delta = 0.0f;
    if (!parseIntToken(strtok(nullptr, " \t"), joint) || !parseFloatToken(strtok(nullptr, " \t"), delta)) {
      Serial.println("[ERR] usage: step <joint> <delta_deg>");
      return;
    }
    if (!validateJointIndex(joint)) return;
    setMotorTargetDeg((size_t)joint, motors[joint].target_deg + delta, true);
    return;
  }

  if (strcmp(cmd, "setmin") == 0 || strcmp(cmd, "setmax") == 0) {
    stopJointGoalSetTest("manual command");
    int joint = -1;
    if (!parseIntToken(strtok(nullptr, " \t"), joint)) {
      Serial.println("[ERR] usage: setmin <joint> OR setmax <joint>");
      return;
    }
    if (!validateJointIndex(joint)) return;

    if (strcmp(cmd, "setmin") == 0) {
      motors[joint].safe_min_deg = clampFloat(motors[joint].current_deg, kAbsoluteMinDeg, kAbsoluteMaxDeg);
      motors[joint].has_captured_min = true;
      if (motors[joint].safe_min_deg > motors[joint].safe_max_deg) {
        motors[joint].safe_max_deg = motors[joint].safe_min_deg;
      }
      Serial.print("[OK] captured min for joint ");
      Serial.print(joint);
      Serial.print(" = ");
      Serial.println(motors[joint].safe_min_deg, 1);
    } else {
      motors[joint].safe_max_deg = clampFloat(motors[joint].current_deg, kAbsoluteMinDeg, kAbsoluteMaxDeg);
      motors[joint].has_captured_max = true;
      if (motors[joint].safe_max_deg < motors[joint].safe_min_deg) {
        motors[joint].safe_min_deg = motors[joint].safe_max_deg;
      }
      Serial.print("[OK] captured max for joint ");
      Serial.print(joint);
      Serial.print(" = ");
      Serial.println(motors[joint].safe_max_deg, 1);
    }
    return;
  }

  if (strcmp(cmd, "save") == 0) {
    saveLimitsToNvs();
    return;
  }

  if (strcmp(cmd, "load") == 0) {
    loadLimitsFromNvs();
    printLimits();
    return;
  }

  if (strcmp(cmd, "reset_limits") == 0) {
    stopJointGoalSetTest("manual command");
    resetSoftLimits();
    Serial.println("[OK] limits reset to calibrated defaults");
    return;
  }

  if (strcmp(cmd, "export") == 0) {
    exportLimitsForFirmware();
    return;
  }

  Serial.print("[ERR] unknown command: ");
  Serial.println(cmd);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      serialLine[serialLineLen] = '\0';
      processCommand(serialLine);
      serialLineLen = 0;
      continue;
    }

    if (serialLineLen < sizeof(serialLine) - 1) {
      serialLine[serialLineLen++] = ch;
    } else {
      serialLineLen = 0;
      Serial.println("[ERR] command too long");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);

  resetSoftLimits();
  for (size_t i = 0; i < kMotorCount; ++i) {
    motors[i].current_deg = kNeutralStartDeg;
    motors[i].target_deg = clampFloat(kStartupPoseDeg[i], motors[i].safe_min_deg, motors[i].safe_max_deg);
    writeMotor(i, motors[i].current_deg);
  }

  loadLimitsFromNvs();
  for (size_t i = 0; i < kMotorCount; ++i) {
    motors[i].target_deg = clampFloat(kStartupPoseDeg[i], motors[i].safe_min_deg, motors[i].safe_max_deg);
  }

  Serial.println();
  Serial.println("RoboFlex Safety Limit Tool started.");
  Serial.println("Type 'help' for commands.");
  Serial.println("Startup pose: moving slowly to calibrated initial targets.");
  printLimits();
}

void loop() {
  handleSerialInput();

  const unsigned long now = millis();
  runJointGoalSetTestStep(now);

  if (now - lastSmoothingMs >= kSmoothingIntervalMs) {
    lastSmoothingMs = now;
    const float smoothing_step = startupPoseInProgress ? kStartupSmoothingStepDeg : kNormalSmoothingStepDeg;

    bool any_moving = false;
    for (size_t i = 0; i < kMotorCount; ++i) {
      const float delta = motors[i].target_deg - motors[i].current_deg;
      if (fabsf(delta) <= smoothing_step) {
        motors[i].current_deg = motors[i].target_deg;
      } else {
        motors[i].current_deg += (delta > 0.0f) ? smoothing_step : -smoothing_step;
      }
      writeMotor(i, motors[i].current_deg);
      any_moving = any_moving || (fabsf(motors[i].target_deg - motors[i].current_deg) > 0.05f);
    }

    digitalWrite(STATUS_LED_PIN, any_moving ? HIGH : LOW);
    if (startupPoseInProgress && !any_moving) {
      startupPoseInProgress = false;
      Serial.println("[OK] Startup pose reached. Normal move speed restored.");
    }
  }
}
