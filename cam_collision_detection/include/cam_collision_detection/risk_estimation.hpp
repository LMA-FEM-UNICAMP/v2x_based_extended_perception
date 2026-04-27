#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

class RiskEstimation
{
  using PosePair = std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>;

private:
  double fatal_threshold;
  double warn_threshold;
  double info_threshold;

public:
  RiskEstimation();
  ~RiskEstimation();


double calculate_trajectories_risk(PredictedPath ego, PredictedPath cv, PosePair& collision_points);

  bool willCollide(geometry_msgs::msg::Pose ego, geometry_msgs::msg::Pose cv);
  double getTimeToCollision(PredictedPath ego, PredictedPath cv, PosePair& collision_points);
};