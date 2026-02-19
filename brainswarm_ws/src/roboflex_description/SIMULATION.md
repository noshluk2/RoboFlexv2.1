# Simulation Guide

This guide runs the robot in Gazebo Sim and connects MoveIt for planning/execution.

## Prerequisites

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
```

Install core packages:

```bash
sudo apt install \
  ros-${ROS_DISTRO:-jazzy}-ros2-control \
  ros-${ROS_DISTRO:-jazzy}-ros2-controllers \
  ros-${ROS_DISTRO:-jazzy}-ros-gz-sim \
  ros-${ROS_DISTRO:-jazzy}-ros-gz-bridge
```

Build and source workspace:

```bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
colcon build --packages-select bras_robot_description roboflex_description roboflex_moveit_config
source install/setup.bash
```

## Launch Order

Terminal 1: simulation

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_description simulation.launch.py
```

Terminal 2: MoveIt `move_group`

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_moveit_config move_group.launch.py
```

Terminal 3: MoveIt RViz

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_moveit_config moveit_rviz.launch.py
```

`use_sim_time` defaults to `true` for both MoveIt launch files.

Verify:

```bash
ros2 param get /move_group use_sim_time
```

## Stop Simulation

Preferred:

- Press `Ctrl+C` in each launch terminal.

Force kill:

```bash
pkill -f "gz sim"
pkill -f "ign gazebo"
pkill -f "ros2 launch roboflex_description simulation.launch.py"
```

