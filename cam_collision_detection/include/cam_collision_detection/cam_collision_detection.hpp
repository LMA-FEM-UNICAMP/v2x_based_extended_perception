#ifndef CAM_COLLISION_DETECTION__HPP_
#define CAM_COLLISION_DETECTION__HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include "cam_collision_detection/decision_making.hpp"
#include "cam_collision_detection/risk_estimation.hpp"

namespace cam_collision_detection
{
class CAMCollisionDetection : public rclcpp::Node
{
  /* Aliases */
  using PredictedPath = autoware_perception_msgs::msg::PredictedPath;
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
  using PosePair = std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>;

public:
  /* Constructor */
  CAMCollisionDetection(const rclcpp::NodeOptions&);

  /* ROS2 Callbacks*/
  void predicted_objects_callback(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);

  /* Methods */

  double getDistance(geometry_msgs::msg::Point ego, geometry_msgs::msg::Point cv);
  
  bool isCVInRange(PredictedObject ego, PredictedObject cv);

  uint16_t getStationID(unique_identifier_msgs::msg::UUID object_id);

private:
  /* ROS2 entities */
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_objects_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr collision_points_pub_;

  /* Collision detection parameters */
  u_int16_t ego_station_id_;
  uint8_t info_threshold_s_;
  uint8_t warn_threshold_s_;
  uint8_t fatal_threshold_s_;
  uint8_t prediction_horizon_s_;
  double collision_threshold_m_;
  double process_distance_threshold_m_;

  risk_politics_t risk_politics_;

  /* Auxiliary configuration*/
  bool debug_;
};
}  // namespace cam_collision_detection
#endif  // CAM_COLLISION_DETECTION__HPP_
