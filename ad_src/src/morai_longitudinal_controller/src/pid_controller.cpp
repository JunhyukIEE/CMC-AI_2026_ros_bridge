#include "morai_longitudinal_controller/pid_controller.hpp"

#include <algorithm>

using std::placeholders::_1;

namespace morai_longitudinal_controller
{

PIDController::PIDController(const rclcpp::NodeOptions & options)
: Node("pid_controller", options),
  vehicle_state_received_(false),
  integral_(0.0),
  previous_error_(0.0)
{
  // Declare parameters
  this->declare_parameter("kp", 0.8);
  this->declare_parameter("ki", 0.0);
  this->declare_parameter("kd", 0.1);
  this->declare_parameter("control_rate", 20.0);

  // Anti-windup limits
  this->declare_parameter("integral_max", 10.0);
  this->declare_parameter("integral_min", -10.0);

  // Output limits
  this->declare_parameter("accel_max", 1.0);
  this->declare_parameter("accel_min", 0.0);
  this->declare_parameter("brake_max", 1.0);
  this->declare_parameter("brake_min", 0.0);

  // Get parameters
  kp_ = this->get_parameter("kp").as_double();
  ki_ = this->get_parameter("ki").as_double();
  kd_ = this->get_parameter("kd").as_double();
  control_rate_ = this->get_parameter("control_rate").as_double();

  integral_max_ = this->get_parameter("integral_max").as_double();
  integral_min_ = this->get_parameter("integral_min").as_double();

  accel_max_ = this->get_parameter("accel_max").as_double();
  accel_min_ = this->get_parameter("accel_min").as_double();
  brake_max_ = this->get_parameter("brake_max").as_double();
  brake_min_ = this->get_parameter("brake_min").as_double();

  // Publishers
  longitudinal_cmd_pub_ = this->create_publisher<morai_msgs::msg::LongitudinalControl>(
    "/longitudinal_cmd", 10);

  // Subscribers
  vehicle_state_sub_ = this->create_subscription<morai_msgs::msg::VehicleState>(
    "/vehicle_state", 10, std::bind(&PIDController::vehicle_state_callback, this, _1));

  // Control timer
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&PIDController::compute_control, this)
  );

  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "PID Longitudinal Controller initialized (kp=%.2f, ki=%.2f, kd=%.2f, rate=%.1f Hz)",
    kp_, ki_, kd_, control_rate_);
}

void PIDController::vehicle_state_callback(
  const morai_msgs::msg::VehicleState::SharedPtr msg)
{
  current_vehicle_state_ = msg;
  vehicle_state_received_ = true;
}

void PIDController::compute_control()
{
  if (!vehicle_state_received_ || !current_vehicle_state_) {
    return;
  }

  // Calculate time delta
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();

  // Avoid division by zero or negative dt
  if (dt <= 0.0) {
    dt = 1.0 / control_rate_;
  }
  last_time_ = current_time;

  // Velocity error: target - current
  double target_velocity = current_vehicle_state_->target_velocity;
  double current_velocity = current_vehicle_state_->velocity;
  double error = target_velocity - current_velocity;

  // Proportional term
  double p_term = kp_ * error;

  // Integral term with anti-windup
  integral_ += error * dt;
  integral_ = std::max(integral_min_, std::min(integral_max_, integral_));
  double i_term = ki_ * integral_;

  // Derivative term
  double derivative = (error - previous_error_) / dt;
  double d_term = kd_ * derivative;
  previous_error_ = error;

  // Total control output
  double control_output = p_term + i_term + d_term;

  // Prepare longitudinal control message
  auto control_msg = morai_msgs::msg::LongitudinalControl();
  control_msg.header.stamp = current_time;
  control_msg.header.frame_id = "base_link";

  // Split control output into accel/brake
  if (control_output > 0.0) {
    // Acceleration
    control_msg.accel = std::max(accel_min_, std::min(accel_max_, control_output));
    control_msg.brake = 0.0;
  } else {
    // Braking
    control_msg.accel = 0.0;
    control_msg.brake = std::max(brake_min_, std::min(brake_max_, std::abs(control_output)));
  }

  // Debug information
  control_msg.velocity_error = error;
  control_msg.acceleration_command = control_output;

  // Publish
  longitudinal_cmd_pub_->publish(control_msg);

  // Log (optional, can be reduced for performance)
  if (static_cast<int>(current_time.seconds()) % 2 == 0) {  // Log every 2 seconds
    RCLCPP_DEBUG(this->get_logger(),
      "Vel: %.2f/%.2f m/s, Error: %.2f, Output: %.3f (accel: %.2f, brake: %.2f)",
      current_velocity, target_velocity, error, control_output,
      control_msg.accel, control_msg.brake);
  }
}

void PIDController::reset_integral()
{
  integral_ = 0.0;
  RCLCPP_INFO(this->get_logger(), "PID integral term reset");
}

}  // namespace morai_longitudinal_controller
