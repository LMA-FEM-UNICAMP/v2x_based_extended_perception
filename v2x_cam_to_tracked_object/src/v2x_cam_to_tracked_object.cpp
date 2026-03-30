#include "autoware/v2x_cam_to_tracked_object/v2x_cam_to_tracked_object.hpp"

#include "etsi_its_msgs_utils/cam_access.hpp"  // access functions

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "etsi_its_cam_msgs/msg/station_type.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

namespace autoware::v2x_cam_to_tracked_object
{
V2XCAM2TrackedObject::V2XCAM2TrackedObject(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("v2x_cam_to_tracked_object", node_options)
{
  RCLCPP_INFO(this->get_logger(), "Starting v2x_cam_to_tracked_object class...");

  // Subscribe to map_projector_info topic
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(
    map_projector_info_sub_, [this](const MapProjectorInfo::Message::ConstSharedPtr msg) {
      callback_map_projector_info(msg);
    });

  cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
    "cam/out", rclcpp::QoS{1},
    std::bind(&V2XCAM2TrackedObject::cam_callback, this, std::placeholders::_1));

  tracked_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/perception/object_recognition/tracking/objects", rclcpp::QoS{1});

  cam_timer_ = this->create_wall_timer(
    std::chrono::microseconds(100), std::bind(&V2XCAM2TrackedObject::cam_timer_callback, this));
    
}

void V2XCAM2TrackedObject::cam_timer_callback()
{
  if (cam_tracked_objects_.objects.size() > 0) {
    cam_tracked_objects_.header.stamp = this->now();
    cam_tracked_objects_.header.frame_id = "map";  // World frame ID

    tracked_objects_pub_->publish(cam_tracked_objects_);
  }
}

void V2XCAM2TrackedObject::callback_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  projector_info_ = *msg;
  received_map_projector_info_ = true;
}

void V2XCAM2TrackedObject::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "CAM RECEIVED!");

  // Return immediately if map_projector_info has not been received yet.
  if (!received_map_projector_info_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "map_projector_info has not been received yet. Check if the map_projection_loader is "
      "successfully launched.");
    return;
  }

  autoware_perception_msgs::msg::TrackedObject cam_tracked_object;

  /// Get pose
  geographic_msgs::msg::GeoPoint cam_gnss;

  cam_gnss.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);
  cam_gnss.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
  cam_gnss.altitude = etsi_its_cam_msgs::access::getAltitude(*msg);

  geometry_msgs::msg::Point cam_position =
    autoware::geography_utils::project_forward(cam_gnss, projector_info_);

  cam_position.z = autoware::geography_utils::convert_height(
    cam_position.z, cam_gnss.latitude, cam_gnss.longitude, MapProjectorInfo::Message::WGS84,
    projector_info_.vertical_datum);

  tf2::Quaternion cam_orientation;
  double yaw =
    M_PI_2 -
    DEG2RAD(etsi_its_cam_msgs::access::getHeading(*msg));  // Converting GNSS heading to ENU

  yaw = atan2(sin(yaw), cos(yaw));

  cam_orientation.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovariance cam_pose_with_covariance{};

  cam_pose_with_covariance.pose.position = cam_position;
  cam_pose_with_covariance.pose.orientation = tf2::toMsg(cam_orientation);

  // Default values for covariance
  cam_pose_with_covariance.covariance[7 * 0] = 10.0;
  cam_pose_with_covariance.covariance[7 * 1] = 10.0;
  cam_pose_with_covariance.covariance[7 * 2] = 10.0;
  cam_pose_with_covariance.covariance[7 * 3] = 0.1;
  cam_pose_with_covariance.covariance[7 * 4] = 0.1;
  cam_pose_with_covariance.covariance[7 * 5] = 1.0;

  cam_tracked_object.kinematics.pose_with_covariance = cam_pose_with_covariance;

  cam_tracked_object.kinematics.orientation_availability = true;

  /// Twist

  cam_tracked_object.kinematics.twist_with_covariance.twist.linear.x =
    etsi_its_cam_msgs::access::getSpeed(*msg);

  cam_tracked_object.kinematics.twist_with_covariance.twist.angular.z =
    DEG2RAD(etsi_its_cam_msgs::access::getYawRate(*msg));

  /// Accel
  try {
    cam_tracked_object.kinematics.acceleration_with_covariance.accel.linear.x =
      etsi_its_cam_msgs::access::getLongitudinalAcceleration(*msg);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  try {
    cam_tracked_object.kinematics.acceleration_with_covariance.accel.linear.y =
      etsi_its_cam_msgs::access::getLateralAcceleration(*msg);  // ? Left is positive
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  /// Set object type
  autoware_perception_msgs::msg::ObjectClassification cam_classification;

  cam_classification.label =
    etsi_to_autoware_object_class(etsi_its_cam_msgs::access::getStationType(*msg));
  cam_classification.probability = 1.0f;
  cam_tracked_object.classification.emplace_back(cam_classification);

  /// Set object ID
  unique_identifier_msgs::msg::UUID cam_uuid;

  uint32_t cam_station_id = etsi_its_cam_msgs::access::getStationID(*msg);

  std::memset(cam_uuid.uuid.data(), 0, cam_uuid.uuid.size());

  cam_uuid.uuid[0] = (cam_station_id >> 24) & 0xFF;
  cam_uuid.uuid[1] = (cam_station_id >> 16) & 0xFF;
  cam_uuid.uuid[2] = (cam_station_id >> 8) & 0xFF;
  cam_uuid.uuid[3] = (cam_station_id) & 0xFF;

  cam_tracked_object.object_id = cam_uuid;

  /// Shape

  autoware_perception_msgs::msg::Shape cam_shape;

  cam_shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  cam_shape.dimensions.x = etsi_its_cam_msgs::access::getVehicleLength(*msg);
  cam_shape.dimensions.y = etsi_its_cam_msgs::access::getVehicleWidth(*msg);
  cam_shape.dimensions.z = getCAMObjectHeight(msg);

  cam_tracked_object.shape = cam_shape;

  /// Adding the CAM object to tracked objects vector

  bool new_object = true;

  for (auto & object : cam_tracked_objects_.objects) {
    if (cam_tracked_object.object_id == object.object_id) {
      object = cam_tracked_object;
      new_object = false;
      break;
    }
  }

  if (new_object) {
    cam_tracked_objects_.objects.emplace_back(cam_tracked_object);
  }
}

double V2XCAM2TrackedObject::getCAMObjectHeight(
  const etsi_its_cam_msgs::msg::CAM::SharedPtr cam)
{
  uint8_t station_type = etsi_its_cam_msgs::access::getStationType(*cam);

  /// Return an average value for each class
  switch (station_type) {
    case etsi_its_cam_msgs::msg::StationType::UNKNOWN:
      return 2.0;
      break;

    case etsi_its_cam_msgs::msg::StationType::PEDESTRIAN:
      return 1.7;
      break;

    case etsi_its_cam_msgs::msg::StationType::CYCLIST:
      return 1.5;
      break;

    // Scooters or tricycles...
    case etsi_its_cam_msgs::msg::StationType::MOPED:
      return 1.5;
      break;

    case etsi_its_cam_msgs::msg::StationType::MOTORCYCLE:
      return 1.5;
      break;

    case etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR:
      return 1.9;
      break;

    case etsi_its_cam_msgs::msg::StationType::BUS:
      return 3.0;
      break;

    case etsi_its_cam_msgs::msg::StationType::LIGHT_TRUCK:
      return 3.0;
      break;

    case etsi_its_cam_msgs::msg::StationType::HEAVY_TRUCK:
      return 4.0;
      break;

    case etsi_its_cam_msgs::msg::StationType::TRAILER:
      return 2.5;
      break;

    case etsi_its_cam_msgs::msg::StationType::SPECIAL_VEHICLES:
      return 2.0;
      break;

    // Urban trains...
    case etsi_its_cam_msgs::msg::StationType::TRAM:
      return 3.5;
      break;

    case etsi_its_cam_msgs::msg::StationType::ROAD_SIDE_UNIT:
      return 0.5;
      break;

    default:
      return 2.0;
      break;
  }
  return 0.0;
}

uint8_t V2XCAM2TrackedObject::etsi_to_autoware_object_class(const uint8_t etsi_station_type)
{
  /*
  ETSI values:
    uint8 MIN = 0
    uint8 MAX = 255
    uint8 UNKNOWN = 0
    uint8 PEDESTRIAN = 1
    uint8 CYCLIST = 2
    uint8 MOPED = 3
    uint8 MOTORCYCLE = 4
    uint8 PASSENGER_CAR = 5
    uint8 BUS = 6
    uint8 LIGHT_TRUCK = 7
    uint8 HEAVY_TRUCK = 8
    uint8 TRAILER = 9
    uint8 SPECIAL_VEHICLES = 10
    uint8 TRAM = 11
    uint8 ROAD_SIDE_UNIT = 15
  Autoware values:
    uint8 UNKNOWN = 0
    uint8 CAR = 1
    uint8 TRUCK = 2
    uint8 BUS = 3
    uint8 TRAILER = 4
    uint8 MOTORCYCLE = 5
    uint8 BICYCLE = 6
    uint8 PEDESTRIAN = 7
    uint8 ANIMAL = 8
    uint8 HAZARD = 9 # Defined as an object that can cause danger to autonomous driving
    uint8 OVER_DRIVABLE = 10 # Defined as an object that can be safely driven over (e.g., leaf)
    uint8 UNDER_DRIVABLE = 11 # Defined as an object that can be safely driven under (e.g.,
  overpass)
  */

  switch (etsi_station_type) {
    case etsi_its_cam_msgs::msg::StationType::UNKNOWN:
      return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;

    case etsi_its_cam_msgs::msg::StationType::PEDESTRIAN:
      return autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      break;

    case etsi_its_cam_msgs::msg::StationType::CYCLIST:
      return autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
      break;

    // Scooters or tricycles...
    case etsi_its_cam_msgs::msg::StationType::MOPED:
      return autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      break;

    case etsi_its_cam_msgs::msg::StationType::MOTORCYCLE:
      return autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      break;

    case etsi_its_cam_msgs::msg::StationType::PASSENGER_CAR:
      return autoware_perception_msgs::msg::ObjectClassification::CAR;
      break;

    case etsi_its_cam_msgs::msg::StationType::BUS:
      return autoware_perception_msgs::msg::ObjectClassification::BUS;
      break;

    case etsi_its_cam_msgs::msg::StationType::LIGHT_TRUCK:
      return autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      break;

    case etsi_its_cam_msgs::msg::StationType::HEAVY_TRUCK:
      return autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      break;

    case etsi_its_cam_msgs::msg::StationType::TRAILER:
      return autoware_perception_msgs::msg::ObjectClassification::TRAILER;
      break;

    case etsi_its_cam_msgs::msg::StationType::SPECIAL_VEHICLES:
      return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;

    // Urban trains...
    case etsi_its_cam_msgs::msg::StationType::TRAM:
      return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;

    case etsi_its_cam_msgs::msg::StationType::ROAD_SIDE_UNIT:
      return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;

    default:
      return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      break;
  }
  return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}
}  // namespace autoware::v2x_cam_to_tracked_object

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::v2x_cam_to_tracked_object::V2XCAM2TrackedObject)
