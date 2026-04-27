

#include "cam_collision_detection/decision_making.hpp"

#include <cmath>

DecisionMaking::DecisionMaking()
{
  this->fatal_threshold = 10.0;
  this->warn_threshold = 15.0;
  this->info_threshold = 20.0;
}
DecisionMaking::DecisionMaking(double fatal_threshold, double warn_threshold, double info_threshold)
{
  this->fatal_threshold = fatal_threshold;
  this->warn_threshold = warn_threshold;
  this->info_threshold = info_threshold;
}

DecisionMaking::~DecisionMaking()
{
}

bool DecisionMaking::risk_assessment(double risk)
{
  if (INFINITY == risk)  // No collision detected
  {
    // RCLCPP_INFO(this->get_logger(), "No collision detected.");
  }
  else  // Possible collision detected
  {
    if (risk < fatal_threshold)  // Level FATAL
    {
      // RCLCPP_FATAL(this->get_logger(), "COLLISION (FATAL ALERT)!!!");
    }
    else if (risk < warn_threshold)  // Level WARN
    {
      // RCLCPP_WARN(this->get_logger(), "COLLISION (WARN ALERT)!!!");
    }
    else if (risk < info_threshold)  // Level INFO
    {
      // RCLCPP_INFO(this->get_logger(), "COLLISION (INFO ALERT)!!!");
    }
    else  // Lowest level
    {
      // RCLCPP_INFO(this->get_logger(), "Vehicles in collision route!!!");
    }

    // RCLCPP_INFO(this->get_logger(), "Time to collision: %.4f s.", risk);

    return true;
  }

  return false;
}