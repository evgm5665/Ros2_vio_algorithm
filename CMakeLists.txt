cmake_minimum_required(VERSION 3.8)
project(vio_alg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   Programs
#     scripts/liveOvioConv.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  PROGRAMS
    scripts/px4_conv.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()