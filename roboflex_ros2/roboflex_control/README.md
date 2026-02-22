# roboflex_control

Hardware interface and ros2_control package for RoboFlex.

## Purpose

- Provides the `roboflex_control/RoboflexUdpHardware` ros2_control plugin.
- Sends one-way motor commands to ESP32 firmware via UDP.
- Owns controller configuration for simulation and real hardware.
- Owns hardware control launches (real hardware and joint position test).

## Plugin Notes

`RoboflexUdpHardware` is intentionally open-loop. It mirrors commanded positions to state interfaces
and sends UDP commands using a change-driven policy plus periodic keepalive.

Tuning parameters from URDF `<hardware><param ...>`:

- `udp_target_ip` (default `255.255.255.255`)
- `udp_target_port` (default `9999`)
- `udp_keepalive_ms` (default `200`)
- `udp_command_change_epsilon_rad` (default `0.0001`)

## Launches

Real hardware control stack:

```bash
ros2 launch roboflex_control real_hardware.launch.py
```

Single-joint position test stack:

```bash
ros2 launch roboflex_control hardware_joint_position_test.launch.py
```

## Useful Arguments

```bash
ros2 launch roboflex_control real_hardware.launch.py firmware_ip:=<esp32_ip> firmware_port:=9999
```

## Controller Configs

- `config/real_hardware_controllers.yaml`
- `config/hardware_joint_position_test_controllers.yaml`
- `config/roboflex_controllers.yaml`
- `config/ros2_controllers.yaml`
