#include "cam_collision_detection/cam_collision_detection.hpp"

#include <autoware_perception_msgs/msg/predicted_path.hpp>

namespace cam_collision_detection
{
CAMCollisionDetection::CAMCollisionDetection(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("cam_collision_detector", node_options)
{
  this->declare_parameter<uint8_t>("info_threshold_s", 15);
  this->declare_parameter<uint8_t>("warn_threshold_s", 10);
  this->declare_parameter<uint8_t>("fatal_threshold_s", 5);
  this->declare_parameter<uint8_t>("prediction_horizon_s", 15);
  this->declare_parameter<double>("collision_threshold_m", 5.0);
  this->declare_parameter<double>("process_distance_threshold_m", 100);
  this->declare_parameter<uint16_t>("ego_station_id", 32);
  this->declare_parameter<bool>("debug", true);  // TODO undo default value

  this->info_threshold_s_ = this->get_parameter("info_threshold_s").get_value<uint8_t>();
  this->warn_threshold_s_ = this->get_parameter("warn_threshold_s").get_value<uint8_t>();
  this->fatal_threshold_s_ = this->get_parameter("fatal_threshold_s").get_value<uint8_t>();
  this->prediction_horizon_s_ = this->get_parameter("prediction_horizon_s").get_value<uint8_t>();
  this->collision_threshold_m_ = this->get_parameter("collision_threshold_m").get_value<double>();
  this->process_distance_threshold_m_ = this->get_parameter("process_distance_threshold_m").get_value<double>();
  this->ego_station_id_ = this->get_parameter("ego_station_id").get_value<uint16_t>();
  this->debug_ = this->get_parameter("debug").get_value<bool>();

  using std::placeholders::_1;
  predicted_objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "/perception/object_recognition/objects", rclcpp::QoS{ 1 },
      std::bind(&CAMCollisionDetection::predicted_objects_callback, this, _1));

  collision_points_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("/collision_detection/collision_poses", rclcpp::QoS{ 1 });
}

void CAMCollisionDetection::predicted_objects_callback(
    const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  rclcpp::Time init_time = this->now();

  autoware_perception_msgs::msg::PredictedObject ego;
  autoware_perception_msgs::msg::PredictedObjects cvs;

  ego.existence_probability = -1.0;

  if (0 == msg->objects.size())
  {
    return;
  }

  for (auto& obj : msg->objects)
  {
    if (debug_)
    {
      ego_station_id_ = getStationID(obj.object_id);
      debug_ = false;
    }

    if (ego_station_id_ == getStationID(obj.object_id))
    {
      ego = obj;
    }
    else
    {
      cvs.objects.emplace_back(obj);
    }
  }

  /// If ego object not received, skip
  if (-1.0 == ego.existence_probability)
  {
    RCLCPP_WARN(this->get_logger(), "EGO object not found, waiting...");
    return;
  }

  /// If CV CAM not received, skip
  if (!cvs.objects.size())
  {
    RCLCPP_WARN(this->get_logger(), "CV objects not received, waiting...");
    return;
  }

  for (auto& cv : cvs.objects)
  {
    geometry_msgs::msg::PoseArray collision_poses_array;
    collision_poses_array.header.stamp = this->now();
    collision_poses_array.header.set__frame_id("map");

    /// If CV is so far away, is not needed to check collision
    if (!isCVInRange(ego, cv))
    {
      RCLCPP_INFO(this->get_logger(), "Vehicles to far away...");
      continue;
    }

    /// Check all CV predicted paths with all ego predicted paths...
    for (auto& cv_path : cv.kinematics.predicted_paths)
    {
      double time_to_collision = INFINITY;
      PosePair collision_points;

      for (auto& ego_path : ego.kinematics.predicted_paths)
      {
        PosePair path_collision_points;
        double path_time_to_collision = getTimeToCollision(ego_path, cv_path, path_collision_points);

        if (path_time_to_collision > 0 && path_time_to_collision < time_to_collision)
        {
          time_to_collision = path_time_to_collision;
          collision_points = path_collision_points;
        }
      }

      if (INFINITY == time_to_collision)
      {
        RCLCPP_INFO(this->get_logger(), "No collision detected.");
        continue;
      }
      else if (time_to_collision < fatal_threshold_s_)
      {
        RCLCPP_FATAL(this->get_logger(), "COLLISION (FATAL ALERT)!!!");
      }
      else if (time_to_collision < warn_threshold_s_)
      {
        RCLCPP_WARN(this->get_logger(), "COLLISION (WARN ALERT)!!!");
      }
      else if (time_to_collision < info_threshold_s_)
      {
        RCLCPP_INFO(this->get_logger(), "COLLISION (INFO ALERT)!!!");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Vehicles in collision route!!!");
      }

      RCLCPP_INFO(this->get_logger(), "Time to collision: %.4f s.", time_to_collision);

      collision_poses_array.poses.emplace_back(collision_points.first);
      collision_poses_array.poses.emplace_back(collision_points.second);
    }

    collision_points_pub_->publish(collision_poses_array);

    rclcpp::Duration elapsed_time = this->now() - init_time;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Elapsed time for collision detection: %lf ms",
                          elapsed_time.nanoseconds() * 1e6);
  }
}

bool CAMCollisionDetection::isCVInRange(PredictedObject ego, PredictedObject cv)
{
  //* If distance is lower the the distance that the CV will drive with the actual speed over the
  //*   prediction horizon, then it is in range.
  return getDistance(ego.kinematics.initial_pose_with_covariance.pose,
                     cv.kinematics.initial_pose_with_covariance.pose) <=
         cv.kinematics.initial_twist_with_covariance.twist.linear.x * prediction_horizon_s_;
}

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

double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv)
{
  for (std::size_t n = 0; n < ego.path.size(); n++)
  {
    double distance = getDistance(ego.path.at(n), cv.path.at(n));

    if (distance < collision_threshold_m_)
    {
      return (ego.time_step.nanosec * 1e-9) * (n + 1);
    }
  }

  return -1.0;
}

double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv, PosePair & collision_points)
{
  for (std::size_t n = 0; n < ego.path.size(); n++)
  {
    double distance = getDistance(ego.path.at(n), cv.path.at(n));

    if (distance < collision_threshold_m_)
    {
      collision_points = std::make_pair(ego.path.at(n), cv.path.at(n));

      RCLCPP_INFO(this->get_logger(), "*** Collision distance: %lf", distance);

      return (ego.time_step.nanosec * 1e-9) * (n + 1);
    }
  }

  return -1.0;
}

double CAMCollisionDetection::getDistance(geometry_msgs::msg::Pose ego, geometry_msgs::msg::Pose cv)
{
  return sqrt((ego.position.x - cv.position.x) * (ego.position.x - cv.position.x) +
              (ego.position.y - cv.position.y) * (ego.position.y - cv.position.y) +
              (ego.position.z - cv.position.z) * (ego.position.z - cv.position.z));
}

bool CAMCollisionDetection::willCollide(double distance)
{
  return distance <= collision_threshold_m_;
}
}  // namespace cam_collision_detection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cam_collision_detection::CAMCollisionDetection)
