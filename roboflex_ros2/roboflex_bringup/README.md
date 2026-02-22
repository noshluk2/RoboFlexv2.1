# roboflex_bringup

Top-level launch package for RoboFlex simulation and real-hardware workflows.

## Purpose

- Starts simulation with optional MoveIt.
- Starts real hardware stack (ros2_control + MoveIt + RViz).
- Provides RViz debug launch for sensor/joint-state inspection.

## Canonical Launches

Simulation + MoveIt:

```bash
ros2 launch roboflex_bringup simulation.launch.py
```

Real hardware + MoveIt:

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py
```

Real hardware entrypoint alias:

```bash
ros2 launch roboflex_bringup real_hardware.launch.py
```

URDF/joint debug RViz:

```bash
ros2 launch roboflex_bringup sensors_rviz.launch.py
```

## Useful Arguments

- `with_moveit:=false` to start only hardware control stack.
- `firmware_ip:=<esp32_ip>` to unicast UDP motor commands to one board.
- `firmware_port:=9999` to set UDP target port (must match firmware).
- `udp_keepalive_ms:=200` UDP keepalive interval when target command is unchanged.
- `udp_command_change_epsilon_rad:=0.0001` minimum command delta that triggers immediate UDP send.
- `start_moveit:=false` in simulation launch to skip MoveIt nodes.
