cmake_minimum_required(VERSION 3.14)
project(v2x_extended_perception)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  scripts
)
