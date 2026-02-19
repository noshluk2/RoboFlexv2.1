# roboflex_moveit_config

MoveIt 2 configuration package for RoboFlex.

## Use This README When

- You are launching MoveIt/RViz for simulation or hardware.
- You are tuning planning speed, kinematics, or controller mapping.

## Key Files

- `config/kinematics.yaml`: kinematics plugin configuration
- `config/joint_limits.yaml`: velocity and acceleration limits used for trajectory parameterization
- `config/moveit_controllers.yaml`: MoveIt controller mapping
- `launch/move_group.launch.py`: starts MoveIt move_group
- `launch/moveit_rviz.launch.py`: starts RViz with MoveIt configuration

## Launch Commands

Simulation mode (default):

```bash
ros2 launch roboflex_moveit_config move_group.launch.py
ros2 launch roboflex_moveit_config moveit_rviz.launch.py
```

Real hardware mode:

```bash
ros2 launch roboflex_moveit_config move_group.launch.py use_sim_time:=false
ros2 launch roboflex_moveit_config moveit_rviz.launch.py use_sim_time:=false
```

Hardware + MoveIt GUI in one launch (recommended for micro-ROS UDP robot):

```bash
ros2 launch roboflex_moveit_config hardware_gui.launch.py
```

## Hardware GUI Workflow

1. Build and source the workspace:

```bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
colcon build --symlink-install
source install/setup.bash
```

2. Start micro-ROS agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

3. In another terminal, source the workspace and run MoveIt + RViz + ros2_control:

```bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_moveit_config hardware_gui.launch.py
```

4. In RViz MotionPlanning panel:
- Select group `arm`
- Plan
- Execute

When executing, ros2_control publishes `std_msgs/Float64MultiArray` on `/motor_command`, matching the firmware subscriber.

Quick checks:

```bash
ros2 topic echo /motor_command
ros2 action list | grep follow_joint_trajectory
```

## Notes

- `use_sim_time` defaults to `true` for `move_group.launch.py` and `moveit_rviz.launch.py`.
- `use_sim_time` defaults to `false` for `hardware_gui.launch.py`.
- If planning fails with timing errors, verify `/joint_states` and `/clock` are being published and that `use_sim_time` matches your workflow.

## Related Guides

- Project doc map: [`../../../README.md`](../../../README.md)
- Real hardware workflow: [`../roboflex_description/REAL_HARDWARE.md`](../roboflex_description/REAL_HARDWARE.md)
- Firmware setup: [`../../../Firmware/README.md`](../../../Firmware/README.md)
