#ifndef V2X_CAM_EXTENDED_PERCEPTION_HPP_
#define V2X_CAM_EXTENDED_PERCEPTION_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include "etsi_its_cam_msgs/msg/cam.hpp"

namespace autoware::v2x_cam_extended_perception
{
class V2XCAMExtendedPerception : public rclcpp::Node
{
  using MapProjectorInfo = autoware::component_interface_specs_universe::map::MapProjectorInfo;

public:
  explicit V2XCAMExtendedPerception();

  void cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);

  void callback_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);

  uint8_t etsi_to_autoware_object_class(const uint8_t etsi_station_type);

  double getCAMObjectHeight(const etsi_its_cam_msgs::msg::CAM::SharedPtr cam);

private:
  bool received_map_projector_info_;
  MapProjectorInfo::Message projector_info_;

  /* ROS2 entities*/
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_sub_;
  autoware::component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr
    map_projector_info_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr tracked_objects_pub_;
};
}  // namespace autoware::v2x_cam_extended_perception
#endif  // V2X_CAM_EXTENDED_PERCEPTION_HPP_
