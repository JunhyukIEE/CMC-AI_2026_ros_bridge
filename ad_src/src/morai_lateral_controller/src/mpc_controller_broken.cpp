#include "morai_lateral_controller/mpc_controller.hpp"

#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>

using std::placeholders::_1;

namespace morai_lateral_controller
{

MPCController::MPCController(const rclcpp::NodeOptions & options)
: Node("mpc_controller", options),
  vehicle_state_received_(false),
  osqp_work_(nullptr),
  osqp_settings_(nullptr),
  osqp_data_(nullptr),
  osqp_initialized_(false),
  previous_steering_(0.0)
{
  // Declare parameters
  this->declare_parameter("wheelbase", 1.04);
  this->declare_parameter("control_rate", 20.0);
  this->declare_parameter("prediction_horizon", 10);
  this->declare_parameter("prediction_dt", 0.1);
  this->declare_parameter("weight_cte", 10.0);
  this->declare_parameter("weight_heading", 1.0);
  this->declare_parameter("weight_steering", 0.1);
  this->declare_parameter("weight_steering_rate", 0.5);
  this->declare_parameter("steering_max", 1.0);
  this->declare_parameter("steering_min", -1.0);

  // Get parameters
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

  // Setup MPC
  if (!setup_mpc()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup MPC solver");
    return;
  }

  // Publishers
  lateral_cmd_pub_ = this->create_publisher<morai_msgs::msg::LateralControl>(
    "/lateral_cmd", 10);

  // Subscribers
  vehicle_state_sub_ = this->create_subscription<morai_msgs::msg::VehicleState>(
    "/vehicle_state", 10, std::bind(&MPCController::vehicle_state_callback, this, _1));

  // Control timer
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&MPCController::compute_control, this)
  );

  RCLCPP_INFO(this->get_logger(),
    "MPC Lateral Controller initialized (N=%d, dt=%.2f s)",
    prediction_horizon_, prediction_dt_);
}

MPCController::~MPCController()
{
  if (osqp_work_) {
    osqp_cleanup(osqp_work_);
  }
}

void MPCController::vehicle_state_callback(
  const morai_msgs::msg::VehicleState::SharedPtr msg)
{
  current_vehicle_state_ = msg;
  vehicle_state_received_ = true;
}

bool MPCController::setup_mpc()
{
  osqp_initialized_ = true;
  return true;
}

bool MPCController::solve_mpc(
  double cte,
  double heading_error,
  double velocity,
  double & steering_output)
{
  const int N = prediction_horizon_;

  // Decision variables: steering inputs [δ_0, δ_1, ..., δ_{N-1}]
  const int n_vars = N;

  // Predict states based on dynamics
  // State: [cte, heading_error]
  double v = std::max(0.5, velocity);  // Minimum velocity for stability
  double L = wheelbase_;
  double dt = prediction_dt_;

  // Build cost function
  // Cost = Σ (w_cte*cte² + w_he*he² + w_δ*δ² + w_Δδ*(δ_k - δ_{k-1})²)

  // Hessian P (n_vars x n_vars) - diagonal + off-diagonal for rate cost
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;

  // Upper triangular CSC format
  P_indptr.push_back(0);

  for (int i = 0; i < N; ++i) {
    // Predict state at step i+1
    std::vector<double> x(2);
    x[0] = cte;
    x[1] = heading_error;

    // Forward simulate to step i
    for (int k = 0; k <= i; ++k) {
      double delta = (k == 0) ? previous_steering_ : 0.0;  // Assume zero for future
      double x_next_0 = x[0] + v * std::sin(x[1]) * dt;
      double x_next_1 = x[1] + (v / L) * delta * dt;
      x[0] = x_next_0;
      x[1] = x_next_1;
    }

    // State cost contribution to control i
    double state_weight = weight_cte_ * v * v * dt * dt + weight_heading_ * (v / L) * (v / L) * dt * dt;

    // Diagonal element: state cost + steering cost + rate cost
    double diag_val = state_weight + weight_steering_;
    if (i > 0) diag_val += weight_steering_rate_;
    if (i < N - 1) diag_val += weight_steering_rate_;

    P_data.push_back(diag_val);
    P_indices.push_back(i);

    // Off-diagonal for rate cost
    if (i < N - 1) {
      P_data.push_back(-weight_steering_rate_);
      P_indices.push_back(i + 1);
    }

    P_indptr.push_back(P_data.size());
  }

  // Gradient q (linear term)
  std::vector<c_float> q(n_vars);
  for (int i = 0; i < N; ++i) {
    q[i] = 0.0;
    if (i == 0) {
      q[i] = -weight_steering_rate_ * previous_steering_;
    }
  }

  // Constraints: input bounds only
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> l;
  std::vector<c_float> u;

  // Identity matrix for bounds
  A_indptr.push_back(0);
  for (int i = 0; i < N; ++i) {
    A_data.push_back(1.0);
    A_indices.push_back(i);
    A_indptr.push_back(A_data.size());

    l.push_back(steering_min_);
    u.push_back(steering_max_);
  }

  // Create CSC matrices
  csc *P = csc_matrix(n_vars, n_vars, P_data.size(),
                      P_data.data(), P_indices.data(), P_indptr.data());
  csc *A = csc_matrix(N, n_vars, A_data.size(),
                      A_data.data(), A_indices.data(), A_indptr.data());

  // Setup OSQP
  OSQPSettings settings;
  OSQPWorkspace *work;
  OSQPData data;

  data.n = n_vars;
  data.m = N;
  data.P = P;
  data.q = q.data();
  data.A = A;
  data.l = l.data();
  data.u = u.data();

  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.max_iter = 2000;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;

  c_int exitflag = osqp_setup(&work, &data, &settings);
  if (exitflag != 0) {
    RCLCPP_ERROR(this->get_logger(), "OSQP setup failed");
    c_free(P);
    c_free(A);
    return false;
  }

  // Solve
  osqp_solve(work);

  bool success = (work->info->status_val == OSQP_SOLVED ||
                  work->info->status_val == OSQP_SOLVED_INACCURATE);

  if (success) {
    steering_output = work->solution->x[0];
  } else {
    RCLCPP_WARN(this->get_logger(), "OSQP solve failed: status %lld",
                work->info->status_val);
    steering_output = previous_steering_;
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(P);
  c_free(A);

  return success;
}

void MPCController::compute_control()
{
  if (!vehicle_state_received_ || !current_vehicle_state_) {
    return;
  }

  double cte = current_vehicle_state_->cross_track_error;
  double heading_error = current_vehicle_state_->heading_error;
  double velocity = current_vehicle_state_->velocity;

  double steering_angle = 0.0;
  if (!solve_mpc(cte, heading_error, velocity, steering_angle)) {
    steering_angle = previous_steering_;
  }

  // MORAI inverts steering
  steering_angle = -steering_angle;
  steering_angle = std::max(steering_min_, std::min(steering_max_, steering_angle));

  previous_steering_ = steering_angle;

  auto control_msg = morai_msgs::msg::LateralControl();
  control_msg.header.stamp = this->now();
  control_msg.header.frame_id = "base_link";
  control_msg.steering_angle = steering_angle;
  control_msg.steering_angle_velocity = 0.0;
  control_msg.curvature = 0.0;
  control_msg.lateral_error = cte;

  lateral_cmd_pub_->publish(control_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "MPC: CTE=%.3f, HE=%.3f, V=%.2f, δ=%.3f",
    cte, heading_error, velocity, steering_angle);
}

}  // namespace morai_lateral_controller
