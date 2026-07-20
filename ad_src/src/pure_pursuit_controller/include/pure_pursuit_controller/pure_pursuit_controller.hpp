#ifndef PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "morai_msgs/msg/ctrl_cmd.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <limits> // For std::numeric_limits

// A struct to hold waypoint data
struct Waypoint {
    double x;
    double y;
    double yaw;
    double velocity;
};

class PurePursuitNode : public rclcpp::Node
{
public:
    explicit PurePursuitNode(const rclcpp::NodeOptions & options);

private:
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    // Core Logic & Helpers
    void load_waypoints(const std::string &path);
    int find_initial_target_point();
    void publish_target_marker(double x, double y);
    double normalize_angle(double angle);

    // ROS 2 Interfaces
    rclcpp::Publisher<morai_msgs::msg::CtrlCmd>::SharedPtr control_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double lookahead_distance_;
    double kp_, ki_, kd_;
    double loop_rate_;

    // Waypoints
    std::vector<Waypoint> waypoints_;

    // Vehicle State
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    bool is_odom_received_ = false;

    // Controller State
    int target_index_ = -1;
    double pid_integral_ = 0.0;
    double pid_previous_error_ = 0.0;
};

#endif // PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
