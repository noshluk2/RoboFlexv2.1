# RoboFlex Hardware Testing Guide (dev + prod)

This file documents how to build/upload/monitor each firmware environment and how to calibrate joint angles safely before switching to ROS control.

## Environments

- `prod`:
  - Source: `src/main.cpp`
  - Purpose: micro-ROS runtime (`/motor_command`)
- `dev`:
  - Source entry: `src/main_dev.cpp`
  - Uses: `test/motor_safety_limits.cpp`
  - Purpose: serial-first hardware bring-up, motor safety limits, angle calibration

## Build, Upload, Monitor Commands

Run from project root:

```bash
cd /home/luqman/repos/RoboFlexv2.1/Firmware/roboflex_pio
```

### Prod commands (ROS firmware)

```bash
# Build prod
pio run -e prod

# Upload prod
pio run -e prod -t upload

# Monitor prod serial
pio device monitor -e prod -b 115200
```

Expected boot logs in `prod`:
- `Starting Micro-ROS ESP32 Node`
- `micro-ROS WiFi SSID: ...`
- `micro-ROS agent: <ip>:<port>`
- `Subscribed to: /motor_command`

### Dev commands (joint safety firmware)

```bash
# Build dev
pio run -e dev

# Upload dev
pio run -e dev -t upload

# Monitor dev serial
pio device monitor -e dev -b 115200
```

Expected boot logs in `dev`:
- `RoboFlex Safety Limit Tool started.`
- `Type 'help' for commands.`

### Default behavior

`platformio.ini` sets `default_envs = prod`, so these use `prod`:

```bash
pio run
pio run -t upload
```

## Dev Serial Commands (Angle Calibration Tool)

Type in serial monitor and press Enter:

- `help`
- `status`
- `limits`
- `m <joint> <deg>`: move with soft limits
- `raw <joint> <deg>`: move ignoring soft limits (`0..180` clamp only)
- `step <joint> <delta_deg>`
- `all <deg>`
- `setmin <joint>`
- `setmax <joint>`
- `save`
- `load`
- `reset_limits` (back to calibrated defaults below)
- `export`
- `center`
- `test_sets` (run 5 joint-goal sets between safe min/max with 3-second gap)
- `test_array` (alias for `test_sets`)
- `stop_test`
- `<j1..j5 or j0..j5>` direct joint goals (space-separated)

Joint indexes map to PCA9685 channels:
- `joint 0 -> ch 0`
- `joint 1 -> ch 1`
- `joint 2 -> ch 2`
- `joint 3 -> ch 3`
- `joint 4 -> ch 4`
- `joint 5 -> ch 5 (gripper)`

## Current Compiled Safety Defaults

- `safe_min` for all joints `0..5`: `0.0`
- `safe_max`:
  - `joint 0: 120.0` (conservative default until explicitly calibrated)
  - `joint 1: 120.0`
  - `joint 2: 80.0`
  - `joint 3: 120.0`
  - `joint 4: 120.0`
  - `joint 5: 120.0` (gripper open max, with `0.0` as closed)
- Startup pose targets:
  - `joint 1` starts at `0.0` (requested change)
  - other joints start at `90.0`
- Startup movement runs at reduced speed, then normal speed resumes automatically.

## Step-by-Step: Test and Set Safe Angles for Each Joint

### 1) Safety prep

- Lift arm clear of table/collisions.
- Use stable external servo power.
- Ensure common GND between ESP32 and servo supply.
- Keep emergency power-off accessible.

### 2) Flash and open monitor in `dev`

```bash
pio run -e dev -t upload
pio device monitor -e dev -b 115200
```

### 3) Reset and center before calibration

In serial monitor:

```text
reset_limits
center
status
```

### 3.0) Manual direct joint-goal input (all together)

You can input a space-separated goal vector directly in serial monitor:

```text
0.0 120.0 120.0 120.0 120.0
```

Behavior:
- 5 values: applies to joints `1..5` (includes `gripper`) and keeps joint `0` unchanged.
- 6 values: applies to joints `0..5` (includes gripper).
- Firmware prints the parsed vector, then applies all targets together.
- Continuous periodic status table logging is disabled; use `status` when needed.

### 3.0.1) Whole motor quick test vectors

Run these vectors from serial monitor for a quick whole-robot movement check:

```text
0.0 0.0 0.0 0.0 0.0
0.0 70.0 90.0 90.0 0.0
```

### 3.1) Quick automated joint-goal set test (3-second gap)

This uses current safety limits and runs **5 different full joint goal sets**.
In each set, all joints receive targets together, then it waits `3s` before the next set.

```text
test_sets
```

To stop while running:

```text
stop_test
```

### 4) Calibrate one joint at a time

Repeat for each joint index (`0..5`):

1. Move close to center:
   - `m <joint> 90`
2. Find minimum safe angle slowly:
   - `step <joint> -5` repeatedly (or `m <joint> <deg>`)
3. When just before mechanical stress/bind, capture min:
   - `setmin <joint>`
4. Return near center:
   - `m <joint> 90`
5. Find maximum safe angle slowly:
   - `step <joint> 5` repeatedly
6. When just before stress/bind, capture max:
   - `setmax <joint>`
7. Verify clamp works:
   - `m <joint> 0`
   - `m <joint> 180`
   - Confirm joint stops at captured limits, not hard endpoints.

### 5) Save limits to NVS

```text
save
status
```

Power-cycle board, then verify persisted values:

```text
load
status
```

### 6) Export calibrated arrays for ROS firmware

```text
export
```

Copy the printed arrays (`kJointMinDeg`, `kJointMaxDeg`) into your production firmware logic once validated.

## Move to Prod after Calibration

After angle/safety calibration is complete:

```bash
pio run -e prod -t upload
pio device monitor -e prod -b 115200
```

Then run ROS command tests on `/motor_command`.
