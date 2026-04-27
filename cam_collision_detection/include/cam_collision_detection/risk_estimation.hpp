#include "rclcpp/rclcpp.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

typedef struct risk_politics
{
  double collision_threshold_m = 0.0;

} risk_politics_t;

class RiskEstimation
{
  using PosePair = std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>;
  using PredictedPath = autoware_perception_msgs::msg::PredictedPath;

private:
  double time_to_collision_;   // Earliest collision time
  PosePair collision_points_;  // Earliest collision points

  geometry_msgs::msg::PoseArray collision_poses_array_;

  risk_politics_t risk_politics_;

  double risk_score_;

public:
  RiskEstimation(risk_politics_t risk_politics);
  ~RiskEstimation();

  geometry_msgs::msg::PoseArray getCollisionPosesArray(rclcpp::Time stamp);
  double getRiskScore();

  double calculate_trajectories_risk(PredictedPath ego, PredictedPath cv);

  bool willCollide(geometry_msgs::msg::Pose ego, geometry_msgs::msg::Pose cv);
  double getTimeToCollision(PredictedPath ego, PredictedPath cv);

  double getDistance(geometry_msgs::msg::Point ego, geometry_msgs::msg::Point cv);
};