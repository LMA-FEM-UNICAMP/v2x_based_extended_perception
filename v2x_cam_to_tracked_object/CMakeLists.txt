cmake_minimum_required(VERSION 3.14)
project(v2x_cam_to_tracked_object)

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

set(V2X_CAM_TO_TRACKED_OBJECT_HEADERS
  include/autoware/v2x_cam_to_tracked_object/v2x_cam_to_tracked_object.hpp
)

ament_auto_add_library(v2x_cam_to_tracked_object_node SHARED
  src/v2x_cam_to_tracked_object.cpp
  ${V2X_CAM_TO_TRACKED_OBJECT_HEADERS}
)

target_link_libraries(v2x_cam_to_tracked_object_node
  Geographic
)

rclcpp_components_register_node(v2x_cam_to_tracked_object_node
  PLUGIN "autoware::v2x_cam_to_tracked_object::V2XCAM2TrackedObject"
  EXECUTABLE v2x_cam_to_tracked_object
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  rviz
)
