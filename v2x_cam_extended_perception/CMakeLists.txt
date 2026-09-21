cmake_minimum_required(VERSION 3.14)
project(v2x_cam_extended_perception)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

set(V2X_CAM_EXTENDED_PERCEPTION_POSER_HEADERS
  include/autoware/v2x_cam_extended_perception/v2x_cam_extended_perception.hpp
)

ament_auto_add_library(v2x_cam_extended_perception_node SHARED
  src/v2x_cam_extended_perception.cpp
  ${V2X_CAM_EXTENDED_PERCEPTION_POSER_HEADERS}
)

target_link_libraries(v2x_cam_extended_perception_node
  Geographic
)

rclcpp_components_register_node(v2x_cam_extended_perception_node
  PLUGIN "autoware::v2x_cam_extended_perception::V2XCAMExtendedPerception"
  EXECUTABLE v2x_cam_extended_perception
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  rviz
)
