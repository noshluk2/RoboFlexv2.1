# RoboFlex v2.1

RoboFlex v2.1 is a ROS 2 robotic arm workspace for simulation and real hardware control with ESP32 + micro-ROS, ros2_control, and MoveIt 2.

![Alt text](/resources/robotflex.png)
### To do
- Hardwar Repeate bility tests
- Moveit pose Fix to real robot
- Adding Gripper  group for moveit
- Test micro ros alternative
## Software Stack


- Operating system: Ubuntu 24.04 LTS
- ROS 2: Jazzy Jalisco
- Simulator: Gazebo Sim (Ignition/GZ)
- Motion planning: MoveIt 2
- Firmware transport: micro-ROS (UDP)
- Languages: C++, Python

## Repository Structure

| Area | Path | Purpose |
| --- | --- | --- |
| Robot description and bringup | [`brainswarm_ws/src/roboflex_description`](brainswarm_ws/src/roboflex_description) | Launch files for simulation and real hardware |
| MoveIt configuration | [`brainswarm_ws/src/roboflex_moveit_config`](brainswarm_ws/src/roboflex_moveit_config) | Planning pipelines, kinematics, controller config |
| Simulation world/assets | [`brainswarm_ws/src/bras_robot_description`](brainswarm_ws/src/bras_robot_description) | World, models, and assets used by simulation |
| Firmware | [`Firmware`](Firmware) | ESP32 micro-ROS firmware |

## Workspace Build

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd brainswarm_ws
colcon build
source install/setup.bash
```

## Recommended Startup Commands

Simulation:

```bash
ros2 launch roboflex_description simulation.launch.py
ros2 launch roboflex_moveit_config move_group.launch.py
ros2 launch roboflex_moveit_config moveit_rviz.launch.py
```

Real hardware (recommended one-command ROS stack):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 launch roboflex_moveit_config hardware_gui.launch.py
```

URDF joint slider debug in RViz (for checking ROS-side joint limits without hardware/controllers):

```bash
ros2 launch roboflex_moveit_config joint_state_debug.launch.py
```
