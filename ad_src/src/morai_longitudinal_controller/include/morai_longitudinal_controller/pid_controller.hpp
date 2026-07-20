#ifndef MORAI_LONGITUDINAL_CONTROLLER__PID_CONTROLLER_HPP_
#define MORAI_LONGITUDINAL_CONTROLLER__PID_CONTROLLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/vehicle_state.hpp"
#include "morai_msgs/msg/longitudinal_control.hpp"

namespace morai_longitudinal_controller
{

class PIDController : public rclcpp::Node
{
public:
  explicit PIDController(const rclcpp::NodeOptions & options);
  ~PIDController() = default;

private:
  // Callbacks
  void vehicle_state_callback(const morai_msgs::msg::VehicleState::SharedPtr msg);

  // Control computation
  void compute_control();

  // Reset integral term (e.g., when target changes significantly)
  void reset_integral();

  // ROS 2 Publishers
  rclcpp::Publisher<morai_msgs::msg::LongitudinalControl>::SharedPtr longitudinal_cmd_pub_;

  // ROS 2 Subscribers
  rclcpp::Subscription<morai_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;

  // Parameters
  double kp_;  // Proportional gain
  double ki_;  // Integral gain
  double kd_;  // Derivative gain
  double control_rate_;  // Control loop rate [Hz]

  // Anti-windup
  double integral_max_;
  double integral_min_;

  // Output limits
  double accel_max_;
  double accel_min_;
  double brake_max_;
  double brake_min_;

  // State
  morai_msgs::msg::VehicleState::SharedPtr current_vehicle_state_;
  bool vehicle_state_received_;

  // PID internal state
  double integral_;
  double previous_error_;
  rclcpp::Time last_time_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace morai_longitudinal_controller

#endif  // MORAI_LONGITUDINAL_CONTROLLER__PID_CONTROLLER_HPP_
