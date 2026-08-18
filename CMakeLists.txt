cmake_minimum_required(VERSION 3.8)
project(vio_alg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/stereo_timing_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()