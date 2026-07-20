#ifndef MORAI_LATERAL_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define MORAI_LATERAL_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/vehicle_state.hpp"
#include "morai_msgs/msg/lateral_control.hpp"

namespace morai_lateral_controller
{

class PurePursuitController : public rclcpp::Node
{
public:
  explicit PurePursuitController(const rclcpp::NodeOptions & options);
  ~PurePursuitController() = default;

private:
  // Callbacks
  void vehicle_state_callback(const morai_msgs::msg::VehicleState::SharedPtr msg);

  // Control computation
  void compute_control();

  // ROS 2 Publishers
  rclcpp::Publisher<morai_msgs::msg::LateralControl>::SharedPtr lateral_cmd_pub_;

  // ROS 2 Subscribers
  rclcpp::Subscription<morai_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;

  // Parameters
  double wheelbase_;         // Vehicle wheelbase [m]
  double steering_gain_;     // Steering gain adjustment
  double control_rate_;      // Control loop rate [Hz]

  // Output limits
  double steering_max_;
  double steering_min_;

  // State
  morai_msgs::msg::VehicleState::SharedPtr current_vehicle_state_;
  bool vehicle_state_received_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace morai_lateral_controller

#endif  // MORAI_LATERAL_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
