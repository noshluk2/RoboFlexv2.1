# Firmware

ESP32 firmware for RoboFlex real hardware control over plain UDP (no micro-ROS).

## Active Project

- PlatformIO project: [`.`](.)
- Main source: [`src/main.cpp`](src/main.cpp)
- PlatformIO config: [`platformio.ini`](platformio.ini)
- Dev/safety sketch: [`src/main_dev.cpp`](src/main_dev.cpp)

## Runtime Protocol

- Transport: UDP (one-way, ROS2 host -> ESP32)
- Listening port: `ROBOFLEX_UDP_LISTEN_PORT` (default `9999`)
- Payload format: `CMD <joint_1> <joint_2> <joint_3> <joint_4> <joint_gripper>`
- Units: radians

Example payload:

```text
CMD 0.000000 -1.570000 -0.174444 0.000000 0.100000
```

## Required Configuration

Set network and UDP port in [`platformio.ini`](platformio.ini):

```ini
build_flags =
  -D ROBOFLEX_WIFI_SSID=\"<ssid>\"
  -D ROBOFLEX_WIFI_PASSWORD=\"<password>\"
  -D ROBOFLEX_UDP_LISTEN_PORT=9999
  -D ROBOFLEX_PCA9685_ADDR=0x40
```

## Build and Upload

```bash
cd ~/ros2_ws/src/RoboFlexv2.1/roboflex_pio
pio run -e prod
pio run -e prod -t upload
pio device monitor -b 115200
```

## Serial Health Checks

On boot you should see:

- `WiFi connected. Local IP: ...`
- `Listening for UDP motor commands on port ...`
- no `[ERROR] PCA9685 not detected on I2C bus.`
- periodic `[STATUS]` lines with increasing `rx_count` when ROS2 is commanding

## Related Guides

- Top-level setup: [`../README.md`](../README.md)
- Real hardware bringup: [`../roboflex_ros2/roboflex_bringup/REAL_HARDWARE.md`](../roboflex_ros2/roboflex_bringup/REAL_HARDWARE.md)
