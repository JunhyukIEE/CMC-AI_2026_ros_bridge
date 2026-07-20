#include "morai_lateral_controller/mpc_controller.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Helper function to normalize angles to [-PI, PI]
double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

using std::placeholders::_1;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace morai_lateral_controller
{

MPCController::MPCController(const rclcpp::NodeOptions & options)
: Node("mpc_controller", options),
  filtered_steering_(0.0),
  vehicle_state_received_(false),
  reference_path_received_(false),
  previous_steering_(0.0)
{
  this->declare_parameter("wheelbase", 1.04);
  this->declare_parameter("control_rate", 20.0);
  this->declare_parameter("prediction_horizon", 10);
  this->declare_parameter("prediction_dt", 0.1);
  this->declare_parameter("weight_cte", 15.0);
  this->declare_parameter("weight_heading", 5.0);
  this->declare_parameter("weight_steering", 1.0);
  this->declare_parameter("weight_steering_rate", 50.0);
  this->declare_parameter("steering_max", 1.0);
  this->declare_parameter("steering_min", -1.0);
  this->declare_parameter("steering_angle_limit_deg", 33.0);
  this->declare_parameter("steering_filter_alpha", 0.3);

  wheelbase_ = this->get_parameter("wheelbase").as_double();
  control_rate_ = this->get_parameter("control_rate").as_double();
  prediction_horizon_ = this->get_parameter("prediction_horizon").as_int();
  prediction_dt_ = this->get_parameter("prediction_dt").as_double();
  weight_cte_ = this->get_parameter("weight_cte").as_double();
  weight_heading_ = this->get_parameter("weight_heading").as_double();
  weight_steering_ = this->get_parameter("weight_steering").as_double();
  weight_steering_rate_ = this->get_parameter("weight_steering_rate").as_double();
  steering_max_ = this->get_parameter("steering_max").as_double();
  steering_min_ = this->get_parameter("steering_min").as_double();
  steering_angle_limit_ = this->get_parameter("steering_angle_limit_deg").as_double() * M_PI / 180.0;
  steering_filter_alpha_ = this->get_parameter("steering_filter_alpha").as_double();

  lateral_cmd_pub_ = this->create_publisher<morai_msgs::msg::LateralControl>(
    "/lateral_cmd", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/mpc_debug_markers", 1);
  vehicle_state_sub_ = this->create_subscription<morai_msgs::msg::VehicleState>(
    "/vehicle_state", 10, std::bind(&MPCController::vehicle_state_callback, this, _1));
  reference_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/mpc_reference_path", 10, std::bind(&MPCController::reference_path_callback, this, _1));

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&MPCController::compute_control, this)
  );

  RCLCPP_INFO(this->get_logger(),
    "MPC Lateral Controller initialized (N=%d, dt=%.2f s, w_cte=%.1f, w_he=%.1f, w_rate=%.1f, filter_alpha=%.2f)",
    prediction_horizon_, prediction_dt_, weight_cte_, weight_heading_, weight_steering_rate_, steering_filter_alpha_);
}

MPCController::~MPCController() = default;

void MPCController::vehicle_state_callback(
  const morai_msgs::msg::VehicleState::SharedPtr msg)
{
  current_vehicle_state_ = msg;
  vehicle_state_received_ = true;
}

void MPCController::reference_path_callback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  reference_path_ = msg;
  reference_path_received_ = true;
}

bool MPCController::solve_mpc(
  double cte,
  double heading_error,
  double velocity,
  double & steering_output)
{
  const int N = prediction_horizon_;
  const int nx = 2;  // State dimension: [cte, heading_error]
  const int nu = 1;  // Input dimension: [steering]

  // Vehicle parameters
  double v = std::max(0.5, std::abs(velocity));
  double L = wheelbase_;
  double dt = prediction_dt_;

  // Linearized bicycle model for error dynamics
  // x = [cte, he]
  // x_dot = [v*sin(he), (v/L)*steer]. Linearized: x_dot = [v*he, (v/L)*steer]
  // x_{k+1} = [1, v*dt; 0, 1] * x_k + [0; v/L*dt] * u_k
  MatrixXd A(nx, nx);
  A << 1.0, v * dt,
    0.0, 1.0;

  MatrixXd B(nx, nu);
  B << 0.0,
    (v / L) * dt;

  // Build state prediction matrices: X = Phi * x0 + Psi * U
  MatrixXd Phi(N * nx, nx);
  MatrixXd Psi(N * nx, N * nu);
  Psi.setZero();

  MatrixXd A_power = MatrixXd::Identity(nx, nx);
  for (int k = 0; k < N; ++k) {
    A_power = A * A_power;
    Phi.block(k * nx, 0, nx, nx) = A_power;

    MatrixXd A_power_temp = MatrixXd::Identity(nx, nx);
    for (int j = k; j >= 0; j--) {
      Psi.block(k * nx, j * nu, nx, nu) = A_power_temp * B;
      A_power_temp = A_power_temp * A;
    }
  }

  // Cost matrices (target is error = 0)
  MatrixXd Q = MatrixXd::Zero(nx, nx);
  Q(0, 0) = weight_cte_;
  Q(1, 1) = weight_heading_;

  MatrixXd Q_bar = MatrixXd::Zero(N * nx, N * nx);
  for (int k = 0; k < N; ++k) {
    Q_bar.block(k * nx, k * nx, nx, nx) = Q;
  }

  MatrixXd R = MatrixXd::Identity(N * nu, N * nu) * weight_steering_;

  // Hessian matrix P
  MatrixXd P_eigen = Psi.transpose() * Q_bar * Psi + R;

  // Add steering rate penalty
  if (weight_steering_rate_ > 1e-6) {
    P_eigen(0, 0) += weight_steering_rate_;
    for (int i = 1; i < N; ++i) {
      P_eigen(i, i) += weight_steering_rate_;
      P_eigen(i - 1, i - 1) += weight_steering_rate_;
      P_eigen(i, i - 1) -= weight_steering_rate_;
      P_eigen(i - 1, i) -= weight_steering_rate_;
    }
  }
  
P_eigen = 0.5 * (P_eigen + P_eigen.transpose());

  // Gradient vector q
  VectorXd x0(nx);
  x0 << cte, heading_error;
  VectorXd q_eigen = Psi.transpose() * Q_bar * Phi * x0;

  // Add steering rate penalty to gradient
  if (weight_steering_rate_ > 1e-6) {
      q_eigen(0) -= weight_steering_rate_ * previous_steering_;
  }

  // OSQP 0.6.x API shipped with ROS 2 Humble
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  P_indptr.push_back(0);
  for (int col = 0; col < N * nu; ++col) {
    for (int row = 0; row <= col; ++row) {
      if (std::abs(P_eigen(row, col)) > 1e-9) {
        P_data.push_back(P_eigen(row, col));
        P_indices.push_back(row);
      }
    }
    P_indptr.push_back(static_cast<c_int>(P_data.size()));
  }

  std::vector<c_float> q(N * nu);
  for (int i = 0; i < N * nu; ++i) {
    q[i] = q_eigen(i);
  }

  // Constraints: l <= A*x <= u (box constraints)
  // This is done by setting A = I, and l, u as the bounds
  std::vector<c_float> A_const_data;
  std::vector<c_int> A_const_indices;
  std::vector<c_int> A_const_indptr;
  A_const_indptr.push_back(0);
  for (int i = 0; i < N * nu; ++i) {
      A_const_data.push_back(1.0);
      A_const_indices.push_back(i);
      A_const_indptr.push_back(static_cast<c_int>(A_const_data.size()));
  }

  std::vector<c_float> l(N * nu);
  std::vector<c_float> u(N * nu);
  for (int i = 0; i < N * nu; ++i) {
    l[i] = -steering_angle_limit_;
    u[i] = steering_angle_limit_;
  }

  // Create CSC matrices for P and A
  csc P_csc;
  P_csc.m = N * nu;
  P_csc.n = N * nu;
  P_csc.nzmax = static_cast<c_int>(P_data.size());
  P_csc.nz = -1;  // CSC format
  P_csc.x = P_data.data();
  P_csc.i = P_indices.data();
  P_csc.p = P_indptr.data();

  csc A_csc;
  A_csc.m = N * nu;
  A_csc.n = N * nu;
  A_csc.nzmax = static_cast<c_int>(A_const_data.size());
  A_csc.nz = -1;  // CSC format
  A_csc.x = A_const_data.data();
  A_csc.i = A_const_indices.data();
  A_csc.p = A_const_indptr.data();

  OSQPData data;
  data.n = N * nu;
  data.m = N * nu;
  data.P = &P_csc;
  data.q = q.data();
  data.A = &A_csc;
  data.l = l.data();
  data.u = u.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.max_iter = 2000;

  OSQPWorkspace * solver = nullptr;
  c_int exitflag = osqp_setup(&solver, &data, &settings);

  if (exitflag != 0 || solver == nullptr) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "OSQP setup failed in solve_mpc");
    return false;
  }

  osqp_solve(solver);

  c_int status_val = solver->info->status_val;
  bool success = (status_val == OSQP_SOLVED || status_val == OSQP_SOLVED_INACCURATE);

  if (success) {
    steering_output = solver->solution->x[0];
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "OSQP failed in solve_mpc (status %s), using previous", solver->info->status);
    steering_output = previous_steering_;
  }

  osqp_cleanup(solver);

  return success;
}


void MPCController::compute_control()
{
  if (!vehicle_state_received_ || !current_vehicle_state_ || !reference_path_received_ ||
    reference_path_->poses.empty())
  {
    return;
  }

  // Extract current state
  const double current_x = current_vehicle_state_->x;
  const double current_y = current_vehicle_state_->y;
  const double current_yaw = current_vehicle_state_->yaw;
  const double velocity = current_vehicle_state_->velocity;

  // Find the closest waypoint in the reference path
  size_t closest_waypoint_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < reference_path_->poses.size(); ++i) {
    const double dx = current_x - reference_path_->poses[i].pose.position.x;
    const double dy = current_y - reference_path_->poses[i].pose.position.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_waypoint_idx = i;
    }
  }

  // Define the path segment for CTE and HE calculation
  size_t p1_idx = closest_waypoint_idx;
  size_t p2_idx = closest_waypoint_idx + 1;
  if (p2_idx >= reference_path_->poses.size()) {
    p1_idx = closest_waypoint_idx -1;
    p2_idx = closest_waypoint_idx;
  }

  const auto & p1 = reference_path_->poses[p1_idx].pose.position;
  const auto & p2 = reference_path_->poses[p2_idx].pose.position;

  // Calculate path segment vector and yaw
  const double path_dx = p2.x - p1.x;
  const double path_dy = p2.y - p1.y;
  const double path_yaw = std::atan2(path_dy, path_dx);

  // Calculate Cross-Track Error (CTE)
  // Vector from path start to vehicle
  const double car_dx = current_x - p1.x;
  const double car_dy = current_y - p1.y;
  // Cross product to find the sign of the error
  const double cross_product = path_dx * car_dy - path_dy * car_dx;
  double cte = std::sqrt(min_dist_sq);
  if (cross_product < 0) {
    cte = -cte;
  }

  // Calculate Heading Error (HE)
  double he = normalize_angle(current_yaw - path_yaw);

  // Call the simpler MPC solver that uses CTE and HE
  double steering_angle_rad = 0.0;
  solve_mpc(cte, he, velocity, steering_angle_rad);

  // --- Process the output for MORAI ---
  const double raw_steering_rad = steering_angle_rad;

  // CRITICAL: Invert steering sign to match MORAI simulator's convention.
  steering_angle_rad = -steering_angle_rad;

  // Scale from radians to normalized [-1, 1]
  const double MAX_STEER_RAD = steering_angle_limit_;
  double steering_angle = steering_angle_rad / MAX_STEER_RAD;
  steering_angle = std::max(-1.0, std::min(1.0, steering_angle));

  // Apply low-pass filter
  filtered_steering_ = steering_filter_alpha_ * steering_angle +
                       (1.0 - steering_filter_alpha_) * filtered_steering_;
  filtered_steering_ = std::max(steering_min_, std::min(steering_max_, filtered_steering_));

  // Update previous steering for rate penalty
  previous_steering_ = raw_steering_rad; // Use raw angle for next iteration's rate penalty

  // Publish control command
  auto control_msg = morai_msgs::msg::LateralControl();
  control_msg.header.stamp = this->now();
  control_msg.header.frame_id = "base_link";
  control_msg.steering_angle = filtered_steering_;
  control_msg.lateral_error = cte; // Publish calculated CTE
  lateral_cmd_pub_->publish(control_msg);


  // --- Publish Visualization Markers ---
  visualization_msgs::msg::MarkerArray marker_array;

  // 1. Path Segment Marker (Line Strip)
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = "odom";
  path_marker.header.stamp = this->now();
  path_marker.ns = "mpc_debug";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.points.push_back(p1);
  path_marker.points.push_back(p2);
  path_marker.scale.x = 0.1; // Line width
  path_marker.color.a = 1.0;
  path_marker.color.r = 0.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 0.0;
  marker_array.markers.push_back(path_marker);

  // 2. CTE Marker (Line from car to path)
  visualization_msgs::msg::Marker cte_marker;
  cte_marker.header = path_marker.header;
  cte_marker.ns = "mpc_debug";
  cte_marker.id = 1;
  cte_marker.type = visualization_msgs::msg::Marker::ARROW;
  cte_marker.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Point car_pos;
  car_pos.x = current_x;
  car_pos.y = current_y;
  // Find projected point on the line for visualization
  double t = (car_dx * path_dx + car_dy * path_dy) / (path_dx * path_dx + path_dy * path_dy);
  t = std::max(0.0, std::min(1.0, t)); // Clamp to segment
  geometry_msgs::msg::Point projected_pos;
  projected_pos.x = p1.x + t * path_dx;
  projected_pos.y = p1.y + t * path_dy;
  cte_marker.points.push_back(car_pos);
  cte_marker.points.push_back(projected_pos);
  cte_marker.scale.x = 0.05; // Shaft diameter
  cte_marker.scale.y = 0.1;  // Head diameter
  cte_marker.color.a = 1.0;
  cte_marker.color.r = 1.0;
  cte_marker.color.g = 0.0;
  cte_marker.color.b = 0.0;
  marker_array.markers.push_back(cte_marker);

  // 3. Heading Error Markers (Vehicle vs Path yaw)
  visualization_msgs::msg::Marker he_marker_veh;
  he_marker_veh.header = path_marker.header;
  he_marker_veh.ns = "mpc_debug";
  he_marker_veh.id = 2;
  he_marker_veh.type = visualization_msgs::msg::Marker::ARROW;
  he_marker_veh.action = visualization_msgs::msg::Marker::ADD;
  he_marker_veh.pose.position = car_pos;
  tf2::Quaternion q_veh;
  q_veh.setRPY(0, 0, current_yaw);
  he_marker_veh.pose.orientation = tf2::toMsg(q_veh);
  he_marker_veh.scale.x = 2.0; // Length
  he_marker_veh.scale.y = 0.1;
  he_marker_veh.scale.z = 0.1;
  he_marker_veh.color.a = 1.0;
  he_marker_veh.color.r = 0.0;
  he_marker_veh.color.g = 0.0;
  he_marker_veh.color.b = 1.0; // Blue for vehicle yaw
  marker_array.markers.push_back(he_marker_veh);

  visualization_msgs::msg::Marker he_marker_path = he_marker_veh;
  he_marker_path.id = 3;
  tf2::Quaternion q_path;
  q_path.setRPY(0, 0, path_yaw);
  he_marker_path.pose.orientation = tf2::toMsg(q_path);
  he_marker_path.color.g = 1.0; // Green for path yaw
  he_marker_path.color.b = 0.0;
  marker_array.markers.push_back(he_marker_path);

  // 4. CTE Text Marker
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = path_marker.header;
  text_marker.ns = "mpc_debug";
  text_marker.id = 4;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position = car_pos;
  text_marker.pose.position.z += 1.0; // Display above the car
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "CTE: " << cte << " m\n" << "HE: " << he * 180.0/M_PI << " deg";
  text_marker.text = ss.str();
  text_marker.scale.z = 0.5;
  text_marker.color.a = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  marker_array.markers.push_back(text_marker);

  marker_pub_->publish(marker_array);
}

}  // namespace morai_lateral_controller
