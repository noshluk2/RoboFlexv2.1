# Firmware

ESP32 micro-ROS firmware for RoboFlex real hardware control.

## Use This README When

- You need to build/upload firmware to the ESP32.
- You need to change Wi-Fi SSID/password or micro-ROS agent IP/port.

## Active Firmware Project

- PlatformIO project: [`.`](.)
- Main source: [`src/main.cpp`](src/main.cpp)
- PlatformIO config: [`platformio.ini`](platformio.ini)
- Legacy Arduino backups: [`test`](test)

## Required Configuration

Set network and agent endpoint in [`platformio.ini`](platformio.ini):

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

## Local Docs

- Hardware configuration and limits: [`hardware_info.md`](hardware_info.md)
- Hardware testing guide: [`test/testing.md`](test/testing.md)

## Runtime Dependency

Start micro-ROS agent on the ROS host before powering/using the robot:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Related Guides

- Real hardware bringup: [`../../src/roboflex_bringup/REAL_HARDWARE.md`](../../src/roboflex_bringup/REAL_HARDWARE.md)
- Hardware joint-test launch: [`../../src/roboflex_control/launch/hardware_joint_position_test.launch.py`](../../src/roboflex_control/launch/hardware_joint_position_test.launch.py)
- MoveIt hardware GUI workflow: [`../../src/roboflex_moveit/README.md`](../../src/roboflex_moveit/README.md)
- Project documentation map: [`../../README.md`](../../README.md)

### Hardware Details
- MG996R x 5 joints
- MG90S x 1 gripper joint
- ESp32
