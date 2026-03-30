cmake_minimum_required(VERSION 3.14)
project(cam_collision_detection)

find_package(autoware_cmake REQUIRED)
autoware_package()


set(CAM_COLLISION_DETECTION_HEADERS
  include/cam_collision_detection/cam_collision_detection.hpp
)

ament_auto_add_library(cam_collision_detection_node SHARED
  src/cam_collision_detection.cpp
  ${CAM_COLLISION_DETECTION_HEADERS}
)

rclcpp_components_register_node(cam_collision_detection_node
  PLUGIN "cam_collision_detection::CAMCollisionDetection"
  EXECUTABLE cam_collision_detection
)


install(
  PROGRAMS
    scripts/ego_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)