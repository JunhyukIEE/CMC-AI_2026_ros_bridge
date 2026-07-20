#include "morai_lateral_controller/pure_pursuit_controller.hpp"

#include <cmath>
#include <algorithm>

using std::placeholders::_1;

namespace morai_lateral_controller
{

PurePursuitController::PurePursuitController(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_controller", options),
  vehicle_state_received_(false)
{
  // Declare parameters
  this->declare_parameter("wheelbase", 1.04);  // [m] Vehicle wheelbase
  this->declare_parameter("steering_gain", 1.0);  // Steering adjustment factor
  this->declare_parameter("control_rate", 20.0);  // [Hz]

  // Output limits
  this->declare_parameter("steering_max", 1.0);
  this->declare_parameter("steering_min", -1.0);

  // Get parameters
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  steering_gain_ = this->get_parameter("steering_gain").as_double();
  control_rate_ = this->get_parameter("control_rate").as_double();

  steering_max_ = this->get_parameter("steering_max").as_double();
  steering_min_ = this->get_parameter("steering_min").as_double();

  // Publishers
  lateral_cmd_pub_ = this->create_publisher<morai_msgs::msg::LateralControl>(
    "/lateral_cmd", 10);

  // Subscribers
  vehicle_state_sub_ = this->create_subscription<morai_msgs::msg::VehicleState>(
    "/vehicle_state", 10, std::bind(&PurePursuitController::vehicle_state_callback, this, _1));

  // Control timer
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&PurePursuitController::compute_control, this)
  );

  RCLCPP_INFO(this->get_logger(),
    "Pure Pursuit Lateral Controller initialized (wheelbase=%.2f m, rate=%.1f Hz)",
    wheelbase_, control_rate_);
}

void PurePursuitController::vehicle_state_callback(
  const morai_msgs::msg::VehicleState::SharedPtr msg)
{
  current_vehicle_state_ = msg;
  vehicle_state_received_ = true;
}

void PurePursuitController::compute_control()
{
  if (!vehicle_state_received_ || !current_vehicle_state_) {
    return;
  }

  // Get current vehicle pose
  double current_x = current_vehicle_state_->x;
  double current_y = current_vehicle_state_->y;
  double current_yaw = current_vehicle_state_->yaw;

  // Get target waypoint position
  double target_x = current_vehicle_state_->target_x;
  double target_y = current_vehicle_state_->target_y;
  double lookahead_distance = current_vehicle_state_->lookahead_distance;

  // Transform target point to vehicle frame
  // 1. Translate to vehicle origin
  double dx = target_x - current_x;
  double dy = target_y - current_y;

  // 2. Rotate to vehicle frame (negative yaw for inverse rotation)
  double local_x = dx * std::cos(-current_yaw) - dy * std::sin(-current_yaw);
  double local_y = dx * std::sin(-current_yaw) + dy * std::cos(-current_yaw);

  // Calculate steering angle using Pure Pursuit formula
  // alpha: angle from vehicle to target point
  double alpha = std::atan2(local_y, local_x);

  // Pure Pursuit steering angle
  // steering = atan((2 * L * sin(alpha)) / ld)
  // where L = wheelbase, ld = lookahead distance
  double steering_angle = std::atan2(
    (2.0 * wheelbase_ * std::sin(alpha)),
    lookahead_distance
  );

  // Apply steering gain (for fine-tuning)
  steering_angle *= steering_gain_;

  // MORAI simulator typically inverts steering
  steering_angle = -steering_angle;

  // Clamp steering angle
  steering_angle = std::max(steering_min_, std::min(steering_max_, steering_angle));

  // Prepare lateral control message
  auto control_msg = morai_msgs::msg::LateralControl();
  control_msg.header.stamp = this->now();
  control_msg.header.frame_id = "base_link";

  control_msg.steering_angle = steering_angle;
  control_msg.steering_angle_velocity = 0.0;  // Not used for now

  // Debug information
  control_msg.curvature = 0.0;  // Can be computed if needed
  control_msg.lateral_error = current_vehicle_state_->cross_track_error;

  // Publish
  lateral_cmd_pub_->publish(control_msg);

  // Log (optional)
  RCLCPP_DEBUG(this->get_logger(),
    "Target: (%.2f, %.2f), Local: (%.2f, %.2f), Alpha: %.2f rad, Steering: %.3f",
    target_x, target_y, local_x, local_y, alpha, steering_angle);
}

}  // namespace morai_lateral_controller
