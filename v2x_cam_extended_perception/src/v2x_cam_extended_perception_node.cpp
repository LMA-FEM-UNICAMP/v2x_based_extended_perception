#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "autoware/v2x_cam_extended_perception/v2x_cam_extended_perception.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<autoware::v2x_cam_extended_perception::V2XCAMExtendedPerception>());
    rclcpp::shutdown();
    return 0;
}
