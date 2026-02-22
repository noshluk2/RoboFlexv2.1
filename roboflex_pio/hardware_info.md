# RoboFlex Hardware Info

This file documents the active hardware-side joint limits and their ROS/radian equivalents for RViz debugging.

## Source Of Truth

Current compiled defaults are defined in:
- `Firmware/roboflex_pio/src/main.cpp`
  - `kJointDefaultMinDeg = {0, 0, 0, 0, 0}`
  - `kJointDefaultMaxDeg = {120, 120, 80, 120, 120}`
  - `kJointZeroOffsetDeg = {-90, 0, 0, 0, 0}`
  - `radToServoDeg()` maps ROS radians `[-1.57, 1.57] -> [0, 180]` servo degrees
- `Firmware/roboflex_pio/test/testing.md` repeats the same safety defaults.

Important:
- At runtime, saved NVS limits can override these defaults.
- Use `status` in dev firmware to see currently active min/max before final tuning.

## ROS Joint Order Used By Firmware

`/motor_command` index mapping in `main.cpp`:
1. `joint_1`
2. `joint_2`
3. `joint_3`
4. `joint_4`
5. `joint_gripper`

## Conversion Formula

For joints with zero offset:
- `servo_deg = ((rad + 1.57) / 3.14) * 180`
- `rad = -1.57 + (servo_deg / 180) * 3.14`

For `joint_1` (offset `-90 deg`):
- `servo_out_deg = clamp(servo_deg - 90, 0, 180)`
- Effective reachable range is reduced (details below).

## Joint Limits To Use In RViz (Radian)

These are hardware-equivalent command ranges from the current firmware defaults.

| Joint | Safe servo range (deg) | Zero offset (deg) | Effective ROS rad range | Notes |
|---|---:|---:|---:|---|
| `joint_1` | `0 .. 120` | `-90` | `0.0000 .. 1.5700` | With current offset + rad clamp, max reachable servo is `90 deg` (not `120`). |
| `joint_2` | `0 .. 120` | `0` | `-1.5700 .. 0.5233` | Direct map from servo limits. |
| `joint_3` | `0 .. 80` | `0` | `-1.5700 .. -0.1744` | Narrower motion window than joint_2. |
| `joint_4` | `0 .. 120` | `0` | `-1.5700 .. 0.5233` | Same as joint_2. |
| `joint_gripper` | `0 .. 120` | `0` | `-1.5700 .. 0.5233` | Single motor command joint that drives both finger sliders via mimic. |

## Practical RViz Checkpoints

Use these slider values in `joint_state_publisher_gui` to visually compare against real arm behavior:

- `joint_1`: `0.00`, `0.785`, `1.57`
- `joint_2`: `-1.57`, `-0.523`, `0.00`, `0.523`
- `joint_3`: `-1.57`, `-0.872`, `-0.523`, `-0.174`
- `joint_4`: `-1.57`, `-0.523`, `0.00`, `0.523`

## Gripper Modeling Note

URDF now uses:
- one motor command joint (`joint_gripper`, revolute), and
- two linear finger joints (`joint_gripper_left_finger`, `joint_gripper_right_part`) that mimic that motor.

This preserves linear finger motion in RViz while keeping a single motor command variable.

## Gripper Model (Current URDF)

To match the real mechanism (single motor, two linear fingers):
- `joint_gripper` is the motor joint (revolute), command range `0.0 .. 0.523333` rad (`0 .. 120 deg`).
- `joint_gripper_left_finger` is prismatic and mimics motor with multiplier `-0.011465`.
- `joint_gripper_right_part` is prismatic and mimics motor with multiplier `0.011465`.
- Gripper mount offset is currently `+0.07 m` on X from `link_4` to `gripper_drive_link`.
- Each finger travels up to `0.006 m`, so total opening is `0.012 m` (`1.2 cm`) at `120 deg`.
- Finger links remain on `gripper_drive_link` frame; only slider motion is visualized on finger links.
