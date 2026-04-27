#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

class DecisionMaking
{
private:
  double fatal_threshold;
  double warn_threshold;
  double info_threshold;

public:
  DecisionMaking();
  DecisionMaking(double fatal_threshold, double warn_threshold, double info_threshold);
  ~DecisionMaking();

  bool risk_assessment(double risk);
};