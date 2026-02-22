# roboflex_bringup

Top-level launch package for RoboFlex simulation and real-hardware workflows.

## Purpose

- Starts simulation with optional MoveIt.
- Starts real hardware stack (ros2_control + MoveIt + RViz).
- Provides RViz debug launch for sensor/joint-state inspection.
- Can optionally start a local micro-ROS agent if installed.

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

- `start_micro_ros_agent:=true` to start local agent from this launch (requires `micro_ros_agent` package in your environment).
- `with_moveit:=false` to start only hardware control stack.
- `start_moveit:=false` in simulation launch to skip MoveIt nodes.
