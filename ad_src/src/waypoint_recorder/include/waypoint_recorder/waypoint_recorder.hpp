#ifndef WAYPOINT_RECORDER__WAYPOINT_RECORDER_HPP_
#define WAYPOINT_RECORDER__WAYPOINT_RECORDER_HPP_

#include <fstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class WaypointRecorder : public rclcpp::Node
{
public:
  explicit WaypointRecorder(const rclcpp::NodeOptions & options);
  ~WaypointRecorder();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void write_header();
  void publish_trajectory(const geometry_msgs::msg::Pose & pose, const std_msgs::msg::Header & header);
  void publish_waypoint_marker(const geometry_msgs::msg::Point & position);


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::ofstream csv_file_;
  nav_msgs::msg::Path trajectory_msg_;
  visualization_msgs::msg::MarkerArray marker_array_msg_;
  int marker_id_counter_ = 0;

  // Parameters
  std::string odom_topic_;
  std::string save_path_;
  double distance_threshold_;
  std::string trajectory_topic_;
  std::string marker_topic_;
  std::string frame_id_;

  // State
  bool is_first_point_;
  double last_x_;
  double last_y_;
};

#endif  // WAYPOINT_RECORDER__WAYPOINT_RECORDER_HPP_
