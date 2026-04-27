#ifndef V2X_CAM_TO_TRACKED_OBJECT_HPP_
#define V2X_CAM_TO_TRACKED_OBJECT_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>
#include "etsi_its_cam_msgs/msg/cam.hpp"

#include <unordered_map>

namespace autoware::v2x_cam_to_tracked_object
{
class V2XCAM2TrackedObject : public rclcpp::Node
{
  using MapProjectorInfo = autoware::component_interface_specs_universe::map::MapProjectorInfo;

public:
  explicit V2XCAM2TrackedObject(const rclcpp::NodeOptions& node_options);

  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);

  void callback_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);

  uint8_t etsi_to_autoware_object_class(const uint8_t etsi_station_type);

  double getCAMObjectHeight(const etsi_its_cam_msgs::msg::CAM::SharedPtr cam);

  uint32_t getStationID(unique_identifier_msgs::msg::UUID object_id);

  void getObjectID(uint32_t station_id, unique_identifier_msgs::msg::UUID& can_uuid);

  void cam_timer_callback();

private:
  bool received_map_projector_info_;
  MapProjectorInfo::Message projector_info_;

  autoware_perception_msgs::msg::TrackedObjects cam_tracked_objects_;

  /* ROS2 entities*/
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_sub_;
  autoware::component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr map_projector_info_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr tracked_objects_pub_;
  rclcpp::TimerBase::SharedPtr cam_timer_;

  std::unordered_map<uint32_t, rclcpp::Time> cam_object_life_dict_;
  double cam_message_validity_s_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace autoware::v2x_cam_to_tracked_object
#endif  // V2X_CAM_TO_TRACKED_OBJECT_HPP_
