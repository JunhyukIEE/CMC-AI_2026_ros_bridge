#include "waypoint_follower_base/waypoint_follower_base.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

namespace waypoint_follower
{

WaypointFollowerBase::WaypointFollowerBase(const rclcpp::NodeOptions & options)
: Node("waypoint_follower_base", options),
  odom_received_(false),
  lateral_cmd_received_(false),
  longitudinal_cmd_received_(false),
  last_tracked_waypoint_index_(-1)  // -1 = not initialized
{
  // Declare parameters
  this->declare_parameter("waypoint_file_path", "/home/ljh/workspace/CMC-AI_2026/src/ad_src/src/waypoint_recorder/waypoints.csv");
  this->declare_parameter("lookahead_distance", 2.5);
  this->declare_parameter("state_update_rate", 20.0);
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("frame_id", "odom");
  this->declare_parameter("mpc_prediction_horizon", 10);
  this->declare_parameter("randomize_waypoints", false);
  this->declare_parameter("randomize_lateral_offset_max", 0.5);
  this->declare_parameter("randomize_speed_min", 0.0);
  this->declare_parameter("randomize_speed_max", 11.0);
  this->declare_parameter("randomize_smoothing_window", 1);
  this->declare_parameter("randomize_seed", 0);
  this->declare_parameter("randomize_recompute_yaw", true);

  // Get parameters
  waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  state_update_rate_ = this->get_parameter("state_update_rate").as_double();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  mpc_prediction_horizon_ = this->get_parameter("mpc_prediction_horizon").as_int();
  const bool randomize_waypoints = this->get_parameter("randomize_waypoints").as_bool();
  const double randomize_lateral_offset_max =
    this->get_parameter("randomize_lateral_offset_max").as_double();
  const double randomize_speed_min = this->get_parameter("randomize_speed_min").as_double();
  const double randomize_speed_max = this->get_parameter("randomize_speed_max").as_double();
  const int randomize_smoothing_window =
    this->get_parameter("randomize_smoothing_window").as_int();
  const int randomize_seed = this->get_parameter("randomize_seed").as_int();
  const bool randomize_recompute_yaw =
    this->get_parameter("randomize_recompute_yaw").as_bool();

  // Initialize trajectory manager
  trajectory_manager_ = std::make_unique<TrajectoryManager>();

  // Load waypoints
  TrajectoryManager::RandomizationParams rand_params;
  rand_params.enabled = randomize_waypoints;
  rand_params.lateral_offset_max = randomize_lateral_offset_max;
  rand_params.speed_min = randomize_speed_min;
  rand_params.speed_max = randomize_speed_max;
  rand_params.smoothing_window = randomize_smoothing_window;
  rand_params.seed = randomize_seed;
  rand_params.recompute_yaw = randomize_recompute_yaw;

  if (!trajectory_manager_->load_waypoints(waypoint_file_path_, rand_params)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from: %s", waypoint_file_path_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", trajectory_manager_->get_waypoint_count());
  if (randomize_waypoints) {
    RCLCPP_INFO(
      this->get_logger(),
      "Waypoint randomization enabled: offset_max=%.2f, speed=[%.2f,%.2f], smoothing=%d, seed=%d, recompute_yaw=%s",
      randomize_lateral_offset_max,
      randomize_speed_min,
      randomize_speed_max,
      randomize_smoothing_window,
      randomize_seed,
      randomize_recompute_yaw ? "true" : "false");
  }

  // Publishers
  vehicle_state_pub_ = this->create_publisher<morai_msgs::msg::VehicleState>(
    "/vehicle_state", 10);
  ctrl_cmd_pub_ = this->create_publisher<morai_msgs::msg::CtrlCmd>(
    "/CtrlCmd", 10);

  // Full path visualization with transient_local QoS (late joiners can receive)
  auto qos_transient = rclcpp::QoS(10).transient_local();
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planned_path", qos_transient);

  mpc_reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/mpc_reference_path", 10);
  target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/target_waypoint_marker", 10);
  waypoint_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/waypoint_markers", qos_transient);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&WaypointFollowerBase::odom_callback, this, _1));
  lateral_control_sub_ = this->create_subscription<morai_msgs::msg::LateralControl>(
    "/lateral_cmd", 10, std::bind(&WaypointFollowerBase::lateral_control_callback, this, _1));
  longitudinal_control_sub_ = this->create_subscription<morai_msgs::msg::LongitudinalControl>(
    "/longitudinal_cmd", 10, std::bind(&WaypointFollowerBase::longitudinal_control_callback, this, _1));

  // Timer for state updates
  state_update_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / state_update_rate_),
    std::bind(&WaypointFollowerBase::state_update_timer_callback, this)
  );

  // Timer for path visualization (publish every 1 second for Rviz2)
  path_visualization_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&WaypointFollowerBase::publish_path_visualization, this)
  );

  // Publish path visualization immediately
  publish_path_visualization();

  RCLCPP_INFO(this->get_logger(), "Waypoint Follower Base Node initialized");
}

void WaypointFollowerBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update current pose
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.velocity = std::hypot(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y
  );

  // Extract yaw from quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_pose_.yaw);

  odom_received_ = true;
}

void WaypointFollowerBase::lateral_control_callback(
  const morai_msgs::msg::LateralControl::SharedPtr msg)
{
  latest_lateral_cmd_ = msg;
  lateral_cmd_received_ = true;
}

void WaypointFollowerBase::longitudinal_control_callback(
  const morai_msgs::msg::LongitudinalControl::SharedPtr msg)
{
  latest_longitudinal_cmd_ = msg;
  longitudinal_cmd_received_ = true;
}

void WaypointFollowerBase::state_update_timer_callback()
{
  if (!odom_received_) {
    return;
  }

  // Calculate tracking error
  current_tracking_error_ = trajectory_manager_->calculate_tracking_error(
    current_pose_,
    lookahead_distance_
  );

  // Publish vehicle state
  publish_vehicle_state();

  // Publish MPC reference path
  publish_mpc_reference_path();

  // Publish control command if both lateral and longitudinal commands received
  if (lateral_cmd_received_ && longitudinal_cmd_received_) {
    publish_ctrl_cmd();
  }

  // Publish target marker for visualization
  if (current_tracking_error_.target_waypoint_index >= 0) {
    const auto & target_wp = trajectory_manager_->get_waypoint(
      current_tracking_error_.target_waypoint_index
    );
    publish_target_marker(target_wp.x, target_wp.y);
  }
}

void WaypointFollowerBase::publish_vehicle_state()
{
  auto msg = morai_msgs::msg::VehicleState();
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  msg.x = current_pose_.x;
  msg.y = current_pose_.y;
  msg.yaw = current_pose_.yaw;
  msg.velocity = current_pose_.velocity;

  msg.cross_track_error = current_tracking_error_.cross_track_error;
  msg.heading_error = current_tracking_error_.heading_error;
  msg.current_waypoint_index = current_tracking_error_.closest_waypoint_index;
  msg.target_waypoint_index = current_tracking_error_.target_waypoint_index;
  msg.lookahead_distance = lookahead_distance_;

  // Get target velocity and position from target waypoint
  if (current_tracking_error_.target_waypoint_index >= 0) {
    const auto & target_wp = trajectory_manager_->get_waypoint(
      current_tracking_error_.target_waypoint_index
    );
    msg.target_velocity = target_wp.velocity;
    msg.target_x = target_wp.x;
    msg.target_y = target_wp.y;
    msg.target_yaw = target_wp.yaw;
  } else {
    msg.target_velocity = 0.0;
    msg.target_x = 0.0;
    msg.target_y = 0.0;
    msg.target_yaw = 0.0;
  }

  vehicle_state_pub_->publish(msg);
}

void WaypointFollowerBase::publish_ctrl_cmd()
{
  if (!latest_lateral_cmd_ || !latest_longitudinal_cmd_) {
    return;
  }

  auto msg = morai_msgs::msg::CtrlCmd();
  msg.longl_cmd_type = 1;  // Throttle mode

  // Merge lateral and longitudinal commands
  msg.steering = static_cast<float>(latest_lateral_cmd_->steering_angle);
  msg.accel = static_cast<float>(latest_longitudinal_cmd_->accel);
  msg.brake = static_cast<float>(latest_longitudinal_cmd_->brake);

  ctrl_cmd_pub_->publish(msg);
}

void WaypointFollowerBase::publish_path_visualization()
{
  // Publish entire path
  auto path_msg = nav_msgs::msg::Path();
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = frame_id_;

  const auto & waypoints = trajectory_manager_->get_waypoints();
  for (const auto & wp : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, wp.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);

  // Publish waypoint markers
  auto marker_array = visualization_msgs::msg::MarkerArray();
  for (size_t i = 0; i < waypoints.size(); i += 5) {  // Every 5th waypoint
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = waypoints[i].x;
    marker.pose.position.y = waypoints[i].y;
    marker.pose.position.z = 0.0;

    marker.scale.x = 1.3;
    marker.scale.y = 1.3;
    marker.scale.z = 1.3;

    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_array.markers.push_back(marker);
  }

  waypoint_markers_pub_->publish(marker_array);
}

void WaypointFollowerBase::publish_target_marker(double x, double y)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "target";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.5;

  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  target_marker_pub_->publish(marker);
}

void WaypointFollowerBase::publish_mpc_reference_path()
{
  // Publish MPC reference trajectory:
  // [0] = Current vehicle position (ensures Dist=0 at start!)
  // [1~N] = Future waypoints ahead of vehicle

  if (current_tracking_error_.target_waypoint_index < 0) {
    return;
  }

  auto path_msg = nav_msgs::msg::Path();
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = frame_id_;

  const auto & waypoints = trajectory_manager_->get_waypoints();
  const size_t waypoint_count = waypoints.size();

  // ALL POINTS: Sequential search from last tracked waypoint
  // Prevents jumping to opposite side of rectangular path
  // Search range: last_tracked ±20 indices

  // Initialize last_tracked on first call using closest waypoint
  if (last_tracked_waypoint_index_ < 0) {
    last_tracked_waypoint_index_ = current_tracking_error_.closest_waypoint_index;
    RCLCPP_INFO(this->get_logger(), "Initialized last_tracked_waypoint_index to %d",
                last_tracked_waypoint_index_);
  }

  const int SEARCH_RANGE = 20;

  int best_idx = last_tracked_waypoint_index_;
  double best_dist = std::hypot(waypoints[best_idx].x - current_pose_.x,
                                waypoints[best_idx].y - current_pose_.y);

  // Search backward and forward from last tracked index
  for (int offset = 1; offset <= SEARCH_RANGE; ++offset) {
    // Backward
    int idx_back = (last_tracked_waypoint_index_ - offset + waypoint_count) % waypoint_count;
    double dist_back = std::hypot(waypoints[idx_back].x - current_pose_.x,
                                  waypoints[idx_back].y - current_pose_.y);
    if (dist_back < best_dist) {
      best_dist = dist_back;
      best_idx = idx_back;
    }

    // Forward
    int idx_fwd = (last_tracked_waypoint_index_ + offset) % waypoint_count;
    double dist_fwd = std::hypot(waypoints[idx_fwd].x - current_pose_.x,
                                 waypoints[idx_fwd].y - current_pose_.y);
    if (dist_fwd < best_dist) {
      best_dist = dist_fwd;
      best_idx = idx_fwd;
    }
  }

  // Update last tracked index for next iteration
  last_tracked_waypoint_index_ = best_idx;

  int start_index = best_idx;

  // DEBUG: Log reference path info
  if (start_index >= 0 && start_index < static_cast<int>(waypoint_count)) {
    const auto & ref_wp = waypoints[start_index];
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "RefPath: last_tracked=%d, new_closest=%d, RefWP=[%.1f,%.1f], Veh=[%.1f,%.1f], dist=%.1fm",
      static_cast<int>((last_tracked_waypoint_index_ - 1 + waypoint_count) % waypoint_count),
      start_index, ref_wp.x, ref_wp.y,
      current_pose_.x, current_pose_.y, best_dist);
  }

  // Add N waypoints for MPC reference path
  for (int i = 0; i < mpc_prediction_horizon_; ++i) {
    int idx = (start_index + i) % waypoint_count;
    const auto & wp = waypoints[idx];

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, wp.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose);
  }

  mpc_reference_path_pub_->publish(path_msg);
}

}  // namespace waypoint_follower
