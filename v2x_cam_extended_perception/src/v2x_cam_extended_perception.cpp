#include "autoware/v2x_cam_extended_perception/v2x_cam_extended_perception.hpp"

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
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

namespace autoware::v2x_cam_extended_perception
{
V2XCAMExtendedPerception::V2XCAMExtendedPerception(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("v2x_cam_extended_perception", node_options)
{
  RCLCPP_INFO(this->get_logger(), "Starting v2x_cam_extended_perception class...");

  // Subscribe to map_projector_info topic
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(map_projector_info_sub_,
                   [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { callback_map_projector_info(msg); });

  cam_sub_ = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(
      "cam/out", rclcpp::QoS{ 1 }, std::bind(&V2XCAMExtendedPerception::cam_callback, this, std::placeholders::_1));

  detected_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "/perception/object_recognition/detection/objects", rclcpp::QoS{ 1 });

  received_map_projector_info_ = false;
}

void V2XCAMExtendedPerception::callback_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  projector_info_ = *msg;
  received_map_projector_info_ = true;
}

void V2XCAMExtendedPerception::cam_callback(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "CAM RECEIVED!");

  // Return immediately if map_projector_info has not been received yet.
  if (!received_map_projector_info_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
                         "map_projector_info has not been received yet. Check if the map_projection_loader is "
                         "successfully launched.");
    return;
  }

  autoware_perception_msgs::msg::DetectedObject cam_detected_object;

  /// Get pose
  geographic_msgs::msg::GeoPoint cam_gnss;

  cam_gnss.latitude = etsi_its_cam_msgs::access::getLatitude(*msg);
  cam_gnss.longitude = etsi_its_cam_msgs::access::getLongitude(*msg);
  cam_gnss.altitude = etsi_its_cam_msgs::access::getAltitude(*msg);

  geometry_msgs::msg::Point cam_position = autoware::geography_utils::project_forward(cam_gnss, projector_info_);

  cam_position.z =
      autoware::geography_utils::convert_height(cam_position.z, cam_gnss.latitude, cam_gnss.longitude,
                                                MapProjectorInfo::Message::WGS84, projector_info_.vertical_datum);

  tf2::Quaternion cam_orientation;
  double yaw = M_PI_2 - DEG2RAD(etsi_its_cam_msgs::access::getHeading(*msg));  // Converting GNSS heading to ENU

  yaw = atan2(sin(yaw), cos(yaw));

  cam_orientation.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovariance cam_pose_with_covariance{};

  cam_pose_with_covariance.pose.position = cam_position;
  cam_pose_with_covariance.pose.orientation = tf2::toMsg(cam_orientation);

  cam_detected_object.kinematics.has_position_covariance = true;

  //* BETA

  double semi_major_confidence = msg->cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse
                                     .semi_major_confidence.value /
                                 100.0;

  double semi_minor_confidence = msg->cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse
                                     .semi_minor_confidence.value /
                                 100.0;

  double semi_major_orientation = msg->cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse
                                     .semi_major_orientation.value /
                                 10.0;

  RCLCPP_INFO(this->get_logger(), "semi_major_confidence = %lf", semi_major_confidence);
  RCLCPP_INFO(this->get_logger(), "semi_minor_confidence = %lf", semi_minor_confidence);
  RCLCPP_INFO(this->get_logger(), "semi_major_orientation = %lf", semi_major_orientation);

  // Default values for covariance
  cam_pose_with_covariance.covariance[7 * 0] = semi_major_confidence;
  cam_pose_with_covariance.covariance[7 * 1] = semi_minor_confidence;
  cam_pose_with_covariance.covariance[7 * 2] = 1.0;
  cam_pose_with_covariance.covariance[7 * 3] = 0.1;
  cam_pose_with_covariance.covariance[7 * 4] = 0.1;
  cam_pose_with_covariance.covariance[7 * 5] = 1.0;

  cam_detected_object.kinematics.pose_with_covariance = cam_pose_with_covariance;

  cam_detected_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;

  /// Twist

  cam_detected_object.kinematics.has_twist = true;

  cam_detected_object.kinematics.twist_with_covariance.twist.linear.x = etsi_its_cam_msgs::access::getSpeed(*msg);

  cam_detected_object.kinematics.twist_with_covariance.twist.angular.z =
      DEG2RAD(etsi_its_cam_msgs::access::getYawRate(*msg));

  // ? Twist covariance?

  /// Accel
  try
  {
    // TODO cam_detected_object.kinematics.acceleration_with_covariance.accel.linear.x =
    // TODO    etsi_its_cam_msgs::access::getLongitudinalAcceleration(*msg);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  try
  {
    // TODO cam_detected_object.kinematics.acceleration_with_covariance.accel.linear.y =
    // TODO     etsi_its_cam_msgs::access::getLateralAcceleration(*msg);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  /// Set object type
  autoware_perception_msgs::msg::ObjectClassification cam_classification;

  cam_classification.label = etsi_to_autoware_object_class(etsi_its_cam_msgs::access::getStationType(*msg));
  cam_classification.probability = 1.0f;
  cam_detected_object.classification.emplace_back(cam_classification);

  /// Set object ID
  unique_identifier_msgs::msg::UUID cam_uuid;

  uint32_t cam_station_id = etsi_its_cam_msgs::access::getStationID(*msg);

  std::memset(cam_uuid.uuid.data(), 0, cam_uuid.uuid.size());

  cam_uuid.uuid[0] = (cam_station_id >> 24) & 0xFF;
  cam_uuid.uuid[1] = (cam_station_id >> 16) & 0xFF;
  cam_uuid.uuid[2] = (cam_station_id >> 8) & 0xFF;
  cam_uuid.uuid[3] = (cam_station_id) & 0xFF;

  // TODO cam_detected_object.object_id = cam_uuid;

  /// Shape

  autoware_perception_msgs::msg::Shape cam_shape;

  cam_shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  cam_shape.dimensions.x = etsi_its_cam_msgs::access::getVehicleLength(*msg);
  cam_shape.dimensions.y = etsi_its_cam_msgs::access::getVehicleWidth(*msg);
  cam_shape.dimensions.z = getCAMObjectHeight(msg);

  cam_detected_object.shape = cam_shape;

  /// Publish as detected object

  autoware_perception_msgs::msg::DetectedObjects cam_detected_objects;

  cam_detected_objects.header.stamp = this->now();
  cam_detected_objects.header.frame_id = "map";  // World frame ID
  cam_detected_objects.objects.emplace_back(cam_detected_object);

  detected_objects_pub_->publish(cam_detected_objects);
}

double V2XCAMExtendedPerception::getCAMObjectHeight(const etsi_its_cam_msgs::msg::CAM::SharedPtr cam)
{
  uint8_t station_type = etsi_its_cam_msgs::access::getStationType(*cam);

  /// Return an average value for each class
  switch (station_type)
  {
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

uint8_t V2XCAMExtendedPerception::etsi_to_autoware_object_class(const uint8_t etsi_station_type)
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

  switch (etsi_station_type)
  {
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
}  // namespace autoware::v2x_cam_extended_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::v2x_cam_extended_perception::V2XCAMExtendedPerception)
