#include "cam_collision_detection/cam_collision_detection.hpp"

#include <autoware_perception_msgs/msg/predicted_path.hpp>

namespace cam_collision_detection
{
/**
 * @brief Construct a new CAMCollisionDetection::CAMCollisionDetection object
 *
 * @param node_options
 */
CAMCollisionDetection::CAMCollisionDetection(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("cam_collision_detector", node_options)
{
  /* ROS2 Parameters */

  this->declare_parameter<uint8_t>("info_threshold_s", 15);       // INFO level threshold in seconds
  this->declare_parameter<uint8_t>("warn_threshold_s", 10);       // WARN level threshold in seconds
  this->declare_parameter<uint8_t>("fatal_threshold_s", 5);       // FATAL level threshold in seconds
  this->declare_parameter<uint8_t>("prediction_horizon_s", 20);   // Prediction horizon in seconds
  this->declare_parameter<double>("collision_threshold_m", 2.0);  // Distance threshold for collision in meters
  this->declare_parameter<uint16_t>("ego_station_id", 32);        // StationID of EGO
  this->declare_parameter<bool>("debug", true);                   // Flag for debug mode

  this->info_threshold_s_ = this->get_parameter("info_threshold_s").get_value<uint8_t>();
  this->warn_threshold_s_ = this->get_parameter("warn_threshold_s").get_value<uint8_t>();
  this->fatal_threshold_s_ = this->get_parameter("fatal_threshold_s").get_value<uint8_t>();
  this->prediction_horizon_s_ = this->get_parameter("prediction_horizon_s").get_value<uint8_t>();
  this->collision_threshold_m_ = this->get_parameter("collision_threshold_m").get_value<double>();
  this->ego_station_id_ = this->get_parameter("ego_station_id").get_value<uint16_t>();
  this->debug_ = this->get_parameter("debug").get_value<bool>();

  /* ROS2 Topics */

  using std::placeholders::_1;
  predicted_objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", rclcpp::QoS{ 1 },
      std::bind(&CAMCollisionDetection::predicted_objects_callback, this, _1));

  collision_points_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("/collision_detection/collision_poses", rclcpp::QoS{ 1 });
}

/**
 * @brief Receive predicted object array with EGO and CVs and process predicted paths to detect possible collision
 *
 * @param msg
 */
void CAMCollisionDetection::predicted_objects_callback(
    const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  rclcpp::Time init_time = this->now();

  /// EGO predicted trajectory object
  autoware_perception_msgs::msg::PredictedObject ego;

  /// CVs predicted trajectory array
  autoware_perception_msgs::msg::PredictedObjects cvs;

  ego.existence_probability = -1.0;  // Initial value for EGO detection check

  // Check if received array ins't empty
  if (0 == msg->objects.size())
  {
    return;
  }

  /// Extract EGO and CV objects
  for (auto& obj : msg->objects)
  {
    if (debug_)  // If debug mode, the EGO is the first received object
    {
      ego_station_id_ = getStationID(obj.object_id);
      debug_ = false;
    }

    /// Check if the StationID recovered from the object UUID is the EGO StationID
    if (ego_station_id_ == getStationID(obj.object_id))
    {
      ego = obj;  // Object is the EGO
    }
    else
    {
      cvs.objects.emplace_back(obj);  // Object is a CV
    }
  }

  /// If EGO object not received, skip
  if (-1.0 == ego.existence_probability)  // existence_probability out of range
  {
    RCLCPP_WARN(this->get_logger(), "EGO object not found, waiting...");
    return;
  }

  /// If CV object not received, skip
  if (!cvs.objects.size())
  {
    RCLCPP_WARN(this->get_logger(), "CV objects not received, waiting...");
    return;
  }

  //* For each CV received, compare all its predicted trajectories with all EGO predicted trajectories
  for (auto& cv : cvs.objects)
  {
    // Auxiliary structure to display collision points in RViz.
    geometry_msgs::msg::PoseArray collision_poses_array;
    collision_poses_array.header.stamp = this->now();
    collision_poses_array.header.set__frame_id("map");

    /// If CV is so far away, is not needed to check collision
    if (!isCVInRange(ego, cv))
    {
      RCLCPP_INFO(this->get_logger(), "Vehicles so far away...");
      continue;
    }

    double time_to_collision = INFINITY;  // Earliest collision time
    PosePair collision_points;            // Earliest collision points

    /// Check all CV predicted paths...
    for (auto& cv_path : cv.kinematics.predicted_paths)
    {
      ///  with all ego predicted paths...
      for (auto& ego_path : ego.kinematics.predicted_paths)
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
      }
    }

    //* Given the time to collision between the predicted trajectories, classify the risk level

    if (INFINITY == time_to_collision)  // No collision detected
    {
      RCLCPP_INFO(this->get_logger(), "No collision detected.");
    }
    else  // Possible collision detected
    {
      if (time_to_collision < fatal_threshold_s_)  // Level FATAL
      {
        RCLCPP_FATAL(this->get_logger(), "COLLISION (FATAL ALERT)!!!");
      }
      else if (time_to_collision < warn_threshold_s_)  // Level WARN
      {
        RCLCPP_WARN(this->get_logger(), "COLLISION (WARN ALERT)!!!");
      }
      else if (time_to_collision < info_threshold_s_)  // Level INFO
      {
        RCLCPP_INFO(this->get_logger(), "COLLISION (INFO ALERT)!!!");
      }
      else  // Lowest level
      {
        RCLCPP_INFO(this->get_logger(), "Vehicles in collision route!!!");
      }

      RCLCPP_INFO(this->get_logger(), "Time to collision: %.4f s.", time_to_collision);

      // Add collision points to visualization
      collision_poses_array.poses.emplace_back(collision_points.first);
      collision_poses_array.poses.emplace_back(collision_points.second);
    }

    collision_points_pub_->publish(collision_poses_array);  // Publish collision points for RViz

    rclcpp::Duration elapsed_time = this->now() - init_time;  // Compute elapsed_time for collision processing

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Elapsed time for collision detection: %lf ms",
                          elapsed_time.nanoseconds() * 1e6);
  }
}

/**
 * @brief Verify if CV and EGO are in range to predict collision
 *
 * @param ego
 * @param cv
 * @return true
 * @return false
 */
bool CAMCollisionDetection::isCVInRange(PredictedObject ego, PredictedObject cv)
{
  //* Check if the distance between the EGO and the CV is within the prediction range of trajectories, i.e.,  a simple
  //*   conference in the worst case if the predicted trajectories could intercept. If does not, isn't necessary to
  //*   process the trajectories.

  // Distance between EGO and CV;
  double distance = getDistance(ego.kinematics.initial_pose_with_covariance.pose.position,
                                cv.kinematics.initial_pose_with_covariance.pose.position);

  /// Given prediction horizon and the relative speed between EGO and CV, calculate the max range of predicted
  /// trajectory
  double prediction_range = (cv.kinematics.initial_twist_with_covariance.twist.linear.x +
                             ego.kinematics.initial_twist_with_covariance.twist.linear.x) *
                            prediction_horizon_s_;

  RCLCPP_DEBUG(this->get_logger(), "\nDistance: %lf | Prediction range: %lf", distance, prediction_range);

  /// Return if the distance between the EGO and CV is within the prediction range
  return distance <= prediction_range;
}

/**
 * @brief Recover StationID from PredictedObject UUID
 *
 * @param object_id
 * @return uint16_t
 */
uint16_t CAMCollisionDetection::getStationID(unique_identifier_msgs::msg::UUID object_id)
{
  /* StationID to UUID from v2x_cam_to_tracked_object.cpp:
    cam_uuid.uuid[3] = (cam_station_id >> 24) & 0xFF;
    cam_uuid.uuid[2] = (cam_station_id >> 16) & 0xFF;
    cam_uuid.uuid[1] = (cam_station_id >> 8) & 0xFF;
    cam_uuid.uuid[0] = (cam_station_id) & 0xFF;
  */

  /// UUID to StationID:
  return (object_id.uuid[3] << 24) + (object_id.uuid[2] << 16) + (object_id.uuid[1] << 8) + object_id.uuid[1];
}

//// /**
////   @brief Calculate the time to collision
////
////   @param ego
////   @param cv
////   @return double
////  /
//// double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv)
//// {
////   /// For each pose in the predicted trajectory array of the EGO and CV
////   for (std::size_t n = 0; n < ego.path.size(); n++)
////   {
//     /// Check if the poses at stamp n causes collision
////     if (willCollide(ego.path.at(n), cv.path.at(n)))
////     {
//       /// If yes, compute the time when that occurs based in the stamp n and time_step of prediction
////       return (ego.time_step.nanosec * 1e-9) * (n + 1);
////     }
////   }

////   return INFINITY;
//// }

/**
 * @brief Calculate the time to collision and where it will occur
 *
 * @param ego
 * @param cv
 * @param collision_points
 * @return double
 */
double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv, PosePair& collision_points)
{
  /// For each pose in the predicted trajectory array of the EGO and CV
  for (std::size_t n = 0; n < ego.path.size(); n++)
  {
    /// Check if the poses at stamp n causes collision
    if (willCollide(ego.path.at(n), cv.path.at(n)))
    {
      collision_points = std::make_pair(ego.path.at(n), cv.path.at(n));  // Collect collision poses
      /// If yes, compute the time when that occurs based in the stamp n and time_step of prediction
      return (ego.time_step.nanosec * 1e-9) * (n + 1);
    }
  }

  /// Collision not detected
  return INFINITY;
}

/**
 * @brief Calculate the distance in meters between the ego and a CV
 *
 * @param ego
 * @param cv
 * @return double
 */
double CAMCollisionDetection::getDistance(geometry_msgs::msg::Point ego, geometry_msgs::msg::Point cv)
{
  return sqrt((ego.x - cv.x) * (ego.x - cv.x) + (ego.y - cv.y) * (ego.y - cv.y) + (ego.z - cv.z) * (ego.z - cv.z));
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

  /// Return if collision will happen based in the distance between the points is below a threshold
  return getDistance(ego.position, cv.position) <= collision_threshold_m_;
}
}  // namespace cam_collision_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cam_collision_detection::CAMCollisionDetection)
