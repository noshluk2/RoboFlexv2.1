cmake_minimum_required(VERSION 3.8)
project(roboflex_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md REAL_HARDWARE.md SIMULATION.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
