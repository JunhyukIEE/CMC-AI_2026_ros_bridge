#ifndef MORAI_LATERAL_CONTROLLER__MPC_CONTROLLER_HPP_
#define MORAI_LATERAL_CONTROLLER__MPC_CONTROLLER_HPP_

#include <osqp/osqp.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/vehicle_state.hpp"
#include "morai_msgs/msg/lateral_control.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


namespace morai_lateral_controller
{

class MPCController : public rclcpp::Node
{
public:
  explicit MPCController(const rclcpp::NodeOptions & options);
  ~MPCController();

private:
  // Callbacks
  void vehicle_state_callback(const morai_msgs::msg::VehicleState::SharedPtr msg);
  void reference_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  // Control computation
  void compute_control();

  // MPC solver functions
  bool solve_mpc(
    double cte,
    double heading_error,
    double velocity,
    double & steering_output);

  // ROS 2 Publishers
  rclcpp::Publisher<morai_msgs::msg::LateralControl>::SharedPtr lateral_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ROS 2 Subscribers
  rclcpp::Subscription<morai_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_;

  // Parameters
  double wheelbase_;         // Vehicle wheelbase [m]
  double control_rate_;      // Control loop rate [Hz]

  // MPC parameters
  int prediction_horizon_;   // N: prediction steps
  double prediction_dt_;     // dt: time step for prediction [s]

  // Cost weights
  double weight_cte_;        // Weight for cross-track error
  double weight_heading_;    // Weight for heading error
  double weight_steering_;   // Weight for steering input
  double weight_steering_rate_;  // Weight for steering rate change

  // Output limits
  double steering_max_;
  double steering_min_;
  double steering_angle_limit_;

  // Low-pass filter
  double steering_filter_alpha_;  // Filter coefficient (0-1)
  double filtered_steering_;       // Filtered steering output

  // State
  morai_msgs::msg::VehicleState::SharedPtr current_vehicle_state_;
  bool vehicle_state_received_;

  nav_msgs::msg::Path::SharedPtr reference_path_;
  bool reference_path_received_;

  // Previous control input (for rate constraint)
  double previous_steering_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace morai_lateral_controller

#endif  // MORAI_LATERAL_CONTROLLER__MPC_CONTROLLER_HPP_
