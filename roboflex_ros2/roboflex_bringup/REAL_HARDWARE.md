# Real Hardware Guide

This guide runs RoboFlex against real hardware (ESP32 + micro-ROS + ros2_control + MoveIt).

## Build

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1
colcon build --packages-select roboflex_description roboflex_control roboflex_moveit roboflex_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If Conda `(base)` is active, add `--cmake-clean-cache` on the first rebuild.

## Recommended One-Command Bringup

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py
```

This starts:
- ros2_control hardware stack
- MoveIt `move_group`
- RViz MotionPlanning

By default this does not auto-start the micro-ROS agent.

Start agent separately:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Or auto-start it from bringup (if `micro_ros_agent` is installed in your local workspace/underlay):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py start_micro_ros_agent:=true
```

## Control-only Mode (No MoveIt)

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py with_moveit:=false
```

## Verify

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /motor_command
```

## Sensor/Joint RViz

```bash
ros2 launch roboflex_bringup sensors_rviz.launch.py
```
