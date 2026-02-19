# Firmware

ESP32 micro-ROS firmware for RoboFlex real hardware control.

## Use This README When

- You need to build/upload firmware to the ESP32.
- You need to change Wi-Fi SSID/password or micro-ROS agent IP/port.

## Active Firmware Project

- PlatformIO project: [`roboflex_pio`](roboflex_pio)
- Main source: [`roboflex_pio/src/main.cpp`](roboflex_pio/src/main.cpp)
- PlatformIO config: [`roboflex_pio/platformio.ini`](roboflex_pio/platformio.ini)
- Legacy Arduino backups: [`roboflex_pio/test`](roboflex_pio/test)

## Required Configuration

Set network and agent endpoint in [`roboflex_pio/platformio.ini`](roboflex_pio/platformio.ini):

```ini
build_flags =
  -D ROBOFLEX_WIFI_SSID=\"<ssid>\"
  -D ROBOFLEX_WIFI_PASSWORD=\"<password>\"
  -D ROBOFLEX_AGENT_IP=\"<agent_ip>\"
  -D ROBOFLEX_AGENT_PORT=8888
```

## Build and Upload

```bash
cd Firmware/roboflex_pio
pio run
pio run -t upload
pio device monitor -b 115200
```

## Runtime Dependency

Start micro-ROS agent on the ROS host before powering/using the robot:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Related Guides

- Real hardware bringup: [`../brainswarm_ws/src/roboflex_description/REAL_HARDWARE.md`](../brainswarm_ws/src/roboflex_description/REAL_HARDWARE.md)
- MoveIt hardware GUI workflow: [`../brainswarm_ws/src/roboflex_moveit_config/README.md`](../brainswarm_ws/src/roboflex_moveit_config/README.md)
- Project documentation map: [`../README.md`](../README.md)
