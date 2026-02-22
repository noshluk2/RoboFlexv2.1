# RoboFlex Educational Robotic Arm

RoboFlex is 4-DOF robotic arm with simulation and real hardware control of ros2.

![RoboFlex](resources/robotflex.png)

## Stack

- OS: Ubuntu 24.04 LTS
- ROS 2: Jazzy Jalisco
- Motion planning: MoveIt 2
- Simulation: Gazebo Sim (GZ)
- Firmware transport: micro-ROS over UDP

## Workspace Architecture
- [Description](roboflex_ros2/roboflex_description)
    -  URDF/Xacro, meshes, worlds, models
- [Control](roboflex_ros2/roboflex_control)
    - Hardware interface plugin + ros2_control configs/launches
- [Bringup](roboflex_ros2/roboflex_bringup)
    - Simulation and real-hardware top-level launches
    -  RViz debug, micro-ROS agent
- [MoveIt](roboflex_ros2/roboflex_moveit)
    - MoveIt config and MoveIt-specific launches

## Build Workspace

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1
colcon build
source install/setup.bash
```

## Optional micro-ROS Packages
This repo intentionally does not include micro-ROS packages or git submodules.

- If you want local micro-ROS packages, clone them into your own workspace `src/`.

Example:

```bash
cd ~/repos/RoboFlexv2.1/src
mkdir -p uros
git clone https://github.com/micro-ROS/micro-ROS-Agent.git uros/micro-ROS-Agent
git clone https://github.com/micro-ROS/micro_ros_msgs.git uros/micro_ros_msgs
```

## Canonical Launches

Simulation + MoveIt:

```bash
ros2 launch roboflex_bringup simulation.launch.py
```

Real hardware + MoveIt (recommended):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py
```

Real hardware control-only (no MoveIt):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py with_moveit:=false
```

URDF/joint-limit debug (no hardware required):

```bash
ros2 launch roboflex_bringup sensors_rviz.launch.py
```
