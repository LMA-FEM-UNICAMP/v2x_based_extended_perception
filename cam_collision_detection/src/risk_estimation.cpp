#include "cam_collision_detection/risk_estimation.hpp"

#include <cmath>

double RiskEstimation::calculate_trajectories_risk(PredictedPath ego, PredictedPath cv, PosePair& collision_points)
{
  PosePair path_collision_points;  // Current collision points

  // TODO: Create a score for each pair of trajectories based
  // TODO:  in the confidence for each predicted trajectory.

  /// Get time to collision for the current trajectories
  double path_time_to_collision =
      getTimeToCollision(ego_path, cv_path, path_collision_points);  // Current time to collision

  /// Check if the current trajectories cause the earliest collision
  if (path_time_to_collision < time_to_collision)
  {
    time_to_collision = path_time_to_collision;
    collision_points = path_collision_points;
  }

  return time_to_collision;
}

/**
 * @brief Calculate the time to collision and where it will occur
 *
 * @param ego
 * @param cv
 * @param collision_points
 * @return double
 */
double RiskEstimation::getTimeToCollision(PredictedPath ego, PredictedPath cv, PosePair& collision_points)
{
  for (std::size_t n = 0; n < ego.path.size(); n++)
  {
    if (willCollide(ego.path.at(n), cv.path.at(n)))
    {
      collision_points = std::make_pair(ego.path.at(n), cv.path.at(n));
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
bool CAMCollisionDetection::willCollide(geometry_msgs::msg::Pose ego, geometry_msgs::msg::Pose cv)
{
  // TODO: Use orientation

  return getDistance(ego.position, cv.position) <= collision_threshold_m_;
}