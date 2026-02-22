# Simulation Guide

Run RoboFlex simulation with Gazebo and optional MoveIt.

## Build

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1
colcon build --packages-select roboflex_description roboflex_control roboflex_moveit roboflex_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If Conda `(base)` is active, add `--cmake-clean-cache` on the first rebuild.

## Launch Simulation + MoveIt

```bash
ros2 launch roboflex_bringup simulation.launch.py
```

## Launch Simulation Without MoveIt

```bash
ros2 launch roboflex_bringup simulation.launch.py start_moveit:=false
```

## URDF/Joints RViz Debug

```bash
ros2 launch roboflex_bringup sensors_rviz.launch.py
```

## Stop

```bash
pkill -f "ros2 launch roboflex_bringup simulation.launch.py"
```
