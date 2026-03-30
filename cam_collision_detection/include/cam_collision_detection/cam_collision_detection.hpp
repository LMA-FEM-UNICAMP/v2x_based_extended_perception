#ifndef CAM_COLLISION_DETECTION__HPP_
#define CAM_COLLISION_DETECTION__HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace cam_collision_detection
{
class CAMCollisionDetection : public rclcpp::Node
{
  using PredictedPath = autoware_perception_msgs::msg::PredictedPath;

public:
  CAMCollisionDetection();

  void predicted_objects_callback(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr);

  double getTimeToCollision(PredictedPath, PredictedPath);

  double getDistance(geometry_msgs::msg::Pose, geometry_msgs::msg::Pose);
  bool willCollide(double);
  uint16_t getStationID(unique_identifier_msgs::msg::UUID);

private:
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_sub_;

  u_int16_t ego_station_id_;
  uint8_t info_threshold_s_;
  uint8_t warn_threshold_s_;
  uint8_t fatal_threshold_s_;
  uint8_t prediction_horizon_s_;
  double collision_threshold_m_;
  double process_distance_threshold_m_;
};
}  // namespace cam_collision_detection
#endif  // CAM_COLLISION_DETECTION__HPP_
