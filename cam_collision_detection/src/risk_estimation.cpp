#include "cam_collision_detection/risk_estimation.hpp"

#include <cmath>

RiskEstimation::RiskEstimation(risk_politics_t risk_politics)
{
  time_to_collision_ = INFINITY;
  this->risk_politics_ = risk_politics;
}
RiskEstimation::~RiskEstimation()
{
}

geometry_msgs::msg::PoseArray RiskEstimation::getCollisionPosesArray(rclcpp::Time stamp)
{
  collision_poses_array_.header.stamp = stamp;
  collision_poses_array_.header.set__frame_id("map");
  return collision_poses_array_;  // TODO: Return a SharedPtr
}

double RiskEstimation::getRiskScore()
{
  return risk_score_;
}

double RiskEstimation::calculate_trajectories_risk(PredictedPath ego, PredictedPath cv)
{

  // TODO: Create a score for each pair of trajectories based
  // TODO:  in the confidence for each predicted trajectory.

  /// Get time to collision for the current trajectories
  double path_time_to_collision = getTimeToCollision(ego, cv);  // Current time to collision

  /// Check if the current trajectories cause the earliest collision
  if (path_time_to_collision < time_to_collision_)
  {
    time_to_collision_ = path_time_to_collision;
  }

  if (time_to_collision_ != INFINITY)
  {
    collision_poses_array_.poses.emplace_back(collision_points_.first);
    collision_poses_array_.poses.emplace_back(collision_points_.second);
  }

  risk_score_ = time_to_collision_;

  return risk_score_;
}

/**
 * @brief Calculate the time to collision and where it will occur
 *
 * @param ego
 * @param cv
 * @param collision_points
 * @return double
 */
double RiskEstimation::getTimeToCollision(PredictedPath ego, PredictedPath cv)
{
  for (std::size_t n = 0; n < ego.path.size(); n++)
  {
    if (willCollide(ego.path.at(n), cv.path.at(n)))
    {
      collision_points_ = std::make_pair(ego.path.at(n), cv.path.at(n));
      return (ego.time_step.nanosec * 1e-9) * (n + 1);
    }
  }

  return -1.0;
}

/**
 * @brief Use a policy to verify if the predicted poses are a collision
 *
 * @param ego
 * @param cv
 * @return true
 * @return false
 */
bool RiskEstimation::willCollide(geometry_msgs::msg::Pose ego, geometry_msgs::msg::Pose cv)
{
  // TODO: Use orientation

  return getDistance(ego.position, cv.position) <= risk_politics_.collision_threshold_m;
}

/**
 * @brief Calculate the distance in meters between the ego and a CV
 *
 * @param ego
 * @param cv
 * @return double
 */
double RiskEstimation::getDistance(geometry_msgs::msg::Point ego, geometry_msgs::msg::Point cv)
{
  return sqrt((ego.x - cv.x) * (ego.x - cv.x) + (ego.y - cv.y) * (ego.y - cv.y) + (ego.z - cv.z) * (ego.z - cv.z));
}