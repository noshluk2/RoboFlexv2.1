# Real Hardware Guide

This guide runs RoboFlex against real hardware (ESP32 UDP firmware + ros2_control + MoveIt).

## Build

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd ~/ros2_ws
colcon build --packages-select roboflex_description roboflex_control roboflex_moveit roboflex_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If Conda `(base)` is active, add `--cmake-clean-cache` on the first rebuild.

## Firmware First

Flash firmware from [`roboflex_pio`](../../roboflex_pio):

```bash
cd ~/ros2_ws/src/RoboFlexv2.1/roboflex_pio
pio run -e prod -t upload
pio device monitor -b 115200
```

Check serial output for local IP and UDP listener port.

## Recommended One-Command Bringup

Broadcast mode (default):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py
```

Unicast mode (recommended for a single robot):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py firmware_ip:=<esp32_ip> firmware_port:=9999
```

Optional UDP tuning (for command-rate control):

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py \
  firmware_ip:=<esp32_ip> firmware_port:=9999 \
  udp_keepalive_ms:=200 udp_command_change_epsilon_rad:=0.0001
```

This starts:

- ros2_control hardware stack
- MoveIt `move_group`
- RViz MotionPlanning

## Control-only Mode (No MoveIt)

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py with_moveit:=false firmware_ip:=<esp32_ip>
```

## Verify

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

On firmware serial monitor, verify `[STATUS] rx_count=...` increases while moving joints in RViz.

## Sensor/Joint RViz

```bash
ros2 launch roboflex_bringup sensors_rviz.launch.py
```
