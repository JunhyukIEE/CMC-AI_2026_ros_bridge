#include "waypoint_recorder/waypoint_recorder.hpp"

#include <cmath>
#include <iomanip>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

WaypointRecorder::WaypointRecorder(const rclcpp::NodeOptions & options)
: Node("waypoint_recorder_node", options), is_first_point_(true), last_x_(0.0), last_y_(0.0)
{
  // Parameters
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>(
    "save_path", "/home/ljh/workspace/CMC-AI_2026/src/ad_src/src/waypoint_recorder/waypoints.csv");
  this->declare_parameter<double>("distance_threshold", 1.0);
  this->declare_parameter<std::string>("trajectory_topic", "/full_trajectory");
  this->declare_parameter<std::string>("marker_topic", "/waypoint_markers");
  this->declare_parameter<std::string>("frame_id", "odom");

  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("save_path", save_path_);
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("trajectory_topic", trajectory_topic_);
  this->get_parameter("marker_topic", marker_topic_);
  this->get_parameter("frame_id", frame_id_);

  // Subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&WaypointRecorder::odom_callback, this, _1));

  // Publishers
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(trajectory_topic_, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

  // Initialize Messages
  trajectory_msg_.header.frame_id = frame_id_;

  // Open file
  csv_file_.open(save_path_);
  if (csv_file_.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Successfully opened waypoint file: %s", save_path_.c_str());
    write_header();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", save_path_.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Waypoint Recorder Node has been started.");
  RCLCPP_INFO(this->get_logger(), "Subscribing to odom topic: %s", odom_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing trajectory to: %s", trajectory_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing markers to: %s", marker_topic_.c_str());
}

WaypointRecorder::~WaypointRecorder()
{
  if (csv_file_.is_open()) {
    csv_file_.close();
    RCLCPP_INFO(this->get_logger(), "Waypoint file closed.");
  }
}

void WaypointRecorder::write_header()
{
  if (csv_file_.is_open()) {
    csv_file_ << "x,y,yaw,v" << std::endl;
  }
}

void WaypointRecorder::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;

  // Always publish the full trajectory
  publish_trajectory(msg->pose.pose, msg->header);

  if (is_first_point_) {
    is_first_point_ = false;
  } else {
    double dist = std::sqrt(std::pow(current_x - last_x_, 2) + std::pow(current_y - last_y_, 2));
    if (dist < distance_threshold_) {
      return; // Not far enough yet, do not record.
    }
  }

  // --- If we are here, it's either the first point or we moved more than the threshold ---

  // Convert quaternion to yaw
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Calculate linear speed
  double velocity = std::sqrt(
    std::pow(msg->twist.twist.linear.x, 2) +
    std::pow(msg->twist.twist.linear.y, 2));

  // Write to file
  if (csv_file_.is_open()) {
    csv_file_ << std::fixed << std::setprecision(4)
              << current_x << ","
              << current_y << ","
              << yaw << ","
              << velocity << std::endl;
    RCLCPP_INFO(this->get_logger(), "Recorded waypoint: x=%.2f, y=%.2f, v=%.2f", current_x, current_y, velocity);
  }
  
  // Publish a marker for the recorded waypoint
  publish_waypoint_marker(msg->pose.pose.position);

  // Update last recorded position
  last_x_ = current_x;
  last_y_ = current_y;
}

void WaypointRecorder::publish_trajectory(const geometry_msgs::msg::Pose & pose, const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = frame_id_;
  pose_stamped.pose = pose;

  trajectory_msg_.poses.push_back(pose_stamped);
  trajectory_msg_.header.stamp = this->get_clock()->now();
  trajectory_pub_->publish(trajectory_msg_);
}

void WaypointRecorder::publish_waypoint_marker(const geometry_msgs::msg::Point & position)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "waypoints";
  marker.id = marker_id_counter_++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.pose.position = position;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  
  marker_array_msg_.markers.push_back(marker);
  marker_pub_->publish(marker_array_msg_);
}
