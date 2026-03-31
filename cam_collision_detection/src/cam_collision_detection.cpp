#include "cam_collision_detection/cam_collision_detection.hpp"

#include <autoware_perception_msgs/msg/predicted_path.hpp>

namespace cam_collision_detection
{
CAMCollisionDetection::CAMCollisionDetection() : Node("cam_collision_detection")
{
  this->declare_parameter<uint8_t>("info_threshold_s", 15);
  this->declare_parameter<uint8_t>("warn_threshold_s", 10);
  this->declare_parameter<uint8_t>("fatal_threshold_s", 5);
  this->declare_parameter<uint8_t>("prediction_horizon_s", 15);
  this->declare_parameter<double>("collision_threshold_m", 2.5);
  this->declare_parameter<double>("process_distance_threshold_m", 100);
  this->declare_parameter<uint16_t>("ego_station_id", 32);

  this->info_threshold_s_ = this->get_parameter("info_threshold_s").get_value<uint8_t>();
  this->warn_threshold_s_ = this->get_parameter("warn_threshold_s").get_value<uint8_t>();
  this->fatal_threshold_s_ = this->get_parameter("fatal_threshold_s").get_value<uint8_t>();
  this->prediction_horizon_s_ = this->get_parameter("prediction_horizon_s").get_value<uint8_t>();
  this->collision_threshold_m_ = this->get_parameter("collision_threshold_m").get_value<double>();
  this->process_distance_threshold_m_ = this->get_parameter("process_distance_threshold_m").get_value<double>();
  this->ego_station_id_ = this->get_parameter("ego_station_id").get_value<uint16_t>();

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
  autoware_perception_msgs::msg::PredictedObject ego;
  autoware_perception_msgs::msg::PredictedObjects cvs;

  for (auto& obj : msg->objects)
  {
    if (ego_station_id_ == getStationID(obj.object_id))
    {
      ego = obj;
    }
    else
    {
      cvs.objects.emplace_back(obj);
    }
  }

  geometry_msgs::msg::PoseArray collision_poses_array;

  collision_poses_array.header.stamp = this->now();
  collision_poses_array.header.set__frame_id("map");

  for (auto& cv : cvs.objects)
  {
    /// If CV is so far away, is not needed to check collision
    if (!isCVInRange(ego, cv))
    {
      continue;
    }

    for (auto& cv_path : cv.kinematics.predicted_paths)
    {
      for (auto& ego_path : ego.kinematics.predicted_paths)
      {
        PosePair collision_points;

        double time_to_collision = getTimeToCollision(ego_path, cv_path, collision_points);

        if (time_to_collision < 0.0)
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

        collision_poses_array.poses.emplace_back(collision_points.first);
        collision_poses_array.poses.emplace_back(collision_points.second);
      }
    }
  }

  collision_points_pub_->publish(collision_poses_array);
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
  return (object_id.uuid[3] << 24) + (object_id.uuid[2] << 16) + (object_id.uuid[1] << 8) + object_id.uuid[1];
}

double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv)
{
  double collision_distance;

  for (int n = 0; n < ego.path.size(); n++)
  {
    double distance = getDistance(ego.path.at(n), cv.path.at(n));

    if (distance < collision_threshold_m_)
    {
      return (ego.time_step.nanosec * 1e-9) * (n + 1);
    }
  }

  return -1.0;
}

double CAMCollisionDetection::getTimeToCollision(PredictedPath ego, PredictedPath cv, PosePair collision_points)
{
  double collision_distance;

  for (int n = 0; n < ego.path.size(); n++)
  {
    double distance = getDistance(ego.path.at(n), cv.path.at(n));

    if (distance < collision_threshold_m_)
    {
      collision_points = std::make_pair(ego.path.at(n), cv.path.at(n));

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
