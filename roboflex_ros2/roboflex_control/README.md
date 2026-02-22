# roboflex_control

Hardware interface and ros2_control package for RoboFlex.

## Purpose

- Provides the `roboflex_control/DynamixelHardware` ros2_control plugin.
- Owns controller configuration for simulation and real hardware.
- Owns hardware control launches (real hardware and joint position test).

## Launches

Real hardware control stack:

```bash
ros2 launch roboflex_control real_hardware.launch.py
```

Single-joint position test stack:

```bash
ros2 launch roboflex_control hardware_joint_position_test.launch.py
```

## Controller Configs

- `config/real_hardware_controllers.yaml`
- `config/hardware_joint_position_test_controllers.yaml`
- `config/roboflex_controllers.yaml`
- `config/ros2_controllers.yaml`
