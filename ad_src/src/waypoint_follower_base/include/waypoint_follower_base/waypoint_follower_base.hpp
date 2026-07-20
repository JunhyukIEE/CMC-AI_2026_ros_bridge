#ifndef WAYPOINT_FOLLOWER_BASE__WAYPOINT_FOLLOWER_BASE_HPP_
#define WAYPOINT_FOLLOWER_BASE__WAYPOINT_FOLLOWER_BASE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "morai_msgs/msg/vehicle_state.hpp"
#include "morai_msgs/msg/lateral_control.hpp"
#include "morai_msgs/msg/longitudinal_control.hpp"
#include "morai_msgs/msg/ctrl_cmd.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "waypoint_follower_base/trajectory_manager.hpp"

namespace waypoint_follower
{

class WaypointFollowerBase : public rclcpp::Node
{
public:
  explicit WaypointFollowerBase(const rclcpp::NodeOptions & options);
  ~WaypointFollowerBase() = default;

private:
  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lateral_control_callback(const morai_msgs::msg::LateralControl::SharedPtr msg);
  void longitudinal_control_callback(const morai_msgs::msg::LongitudinalControl::SharedPtr msg);
  void state_update_timer_callback();

  // Publishing functions
  void publish_vehicle_state();
  void publish_ctrl_cmd();
  void publish_path_visualization();
  void publish_target_marker(double x, double y);
  void publish_mpc_reference_path();

  // Trajectory manager
  std::unique_ptr<TrajectoryManager> trajectory_manager_;

  // ROS 2 Publishers
  rclcpp::Publisher<morai_msgs::msg::VehicleState>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<morai_msgs::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_reference_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;

  // ROS 2 Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<morai_msgs::msg::LateralControl>::SharedPtr lateral_control_sub_;
  rclcpp::Subscription<morai_msgs::msg::LongitudinalControl>::SharedPtr longitudinal_control_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr state_update_timer_;
  rclcpp::TimerBase::SharedPtr path_visualization_timer_;

  // Parameters
  std::string waypoint_file_path_;
  double lookahead_distance_;
  double state_update_rate_;
  std::string odom_topic_;
  std::string frame_id_;
  int mpc_prediction_horizon_;  // Number of future waypoints for MPC

  // Vehicle state
  VehiclePose current_pose_;
  bool odom_received_;

  // Control commands
  morai_msgs::msg::LateralControl::SharedPtr latest_lateral_cmd_;
  morai_msgs::msg::LongitudinalControl::SharedPtr latest_longitudinal_cmd_;
  bool lateral_cmd_received_;
  bool longitudinal_cmd_received_;

  // Tracking state
  TrackingError current_tracking_error_;

  // Last tracked waypoint index for sequential search (prevents jumping to opposite side)
  int last_tracked_waypoint_index_;
};

}  // namespace waypoint_follower

#endif  // WAYPOINT_FOLLOWER_BASE__WAYPOINT_FOLLOWER_BASE_HPP_
