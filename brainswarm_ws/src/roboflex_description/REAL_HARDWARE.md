# Real Hardware Guide

This guide runs the arm against real hardware (ESP32 + micro-ROS + ros2_control).

## Prerequisites

- ESP32 firmware configured and flashed
- Robot powered and connected to the same network as the ROS host
- Workspace built and sourced

Build/source:

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
colcon build --packages-select roboflex_description roboflex_moveit_config
source install/setup.bash
```

## Firmware Configuration

Edit:

- [`Firmware/roboflex_pio/platformio.ini`](../../../Firmware/roboflex_pio/platformio.ini)
- [`Firmware/roboflex_pio/src/main.cpp`](../../../Firmware/roboflex_pio/src/main.cpp)

Set Wi-Fi and agent endpoint using PlatformIO build flags in `platformio.ini`:

- `ROBOFLEX_WIFI_SSID`
- `ROBOFLEX_WIFI_PASSWORD`
- `ROBOFLEX_AGENT_IP`
- `ROBOFLEX_AGENT_PORT`

## Launch Order (Jazzy)

Do not run simulation and real hardware launches at the same time.

- Avoid mixing:
  - `ros2 launch bras_robot_description simulation.launch.py`
  - `ros2 launch roboflex_description real_hardware.launch.py`

Pick one mode per session.

Terminal 1: micro-ROS agent

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Terminal 2: real hardware interface

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_description real_hardware.launch.py
```
This launch now starts:
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `arm_controller`

Terminal 3: MoveIt `move_group` (real clock)

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_moveit_config move_group.launch.py use_sim_time:=false
```

Terminal 4: RViz (real clock)

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/repos/RoboFlexv2.1/brainswarm_ws
source install/setup.bash
ros2 launch roboflex_moveit_config moveit_rviz.launch.py use_sim_time:=false
```

Optional: single-command stack for hardware + MoveIt + RViz:

```bash
ros2 launch roboflex_moveit_config hardware_gui.launch.py
```

Verify clock mode:

```bash
ros2 param get /move_group use_sim_time
```

Expected value for real hardware: `False`.

Verify controllers:

```bash
ros2 control list_controllers
```

Expected active controllers:
- `joint_state_broadcaster`
- `arm_controller`
