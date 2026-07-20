#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm> // For std::min, std::max
#include <sstream>   // For std::istringstream

using std::placeholders::_1;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_node", options)
{
    // Parameters
    this->declare_parameter("waypoint_path", "/home/ljh/workspace/CMC-AI_2026/src/ad_src/src/waypoint_recorder/waypoints.csv");
    this->declare_parameter("lookahead_distance", 2.5);
    this->declare_parameter("kp", 0.8);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.1);
    this->declare_parameter("loop_rate_hz", 20.0);
    
    // Get parameters
    std::string waypoint_path = this->get_parameter("waypoint_path").as_string();
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    loop_rate_ = this->get_parameter("loop_rate_hz").as_double();

    // Publishers and Subscribers
    control_pub_ = this->create_publisher<morai_msgs::msg::CtrlCmd>("/CtrlCmd", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PurePursuitNode::odom_callback, this, _1));
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit_target", 10);

    // Load Waypoints
    load_waypoints(waypoint_path);
    if (waypoints_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Waypoints file failed to load or is empty. Shutting down.");
        // rclcpp::shutdown(); // Don't call shutdown in constructor, it will crash the application
        return; // Node will remain active but won't do anything
    }
    RCLCPP_INFO(this->get_logger(), "%lu waypoints loaded.", waypoints_.size());

    // Start the control loop timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate_),
        std::bind(&PurePursuitNode::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Node (C++) has been started.");
}

void PurePursuitNode::load_waypoints(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open waypoint file: %s", path.c_str());
        return;
    }

    std::string line;
    std::getline(file, line); // Skip header line
    
    while (std::getline(file, line)) {
        Waypoint wp;
        char comma;
        std::istringstream s(line);
        s >> wp.x >> comma >> wp.y >> comma >> wp.yaw >> comma >> wp.velocity;
        waypoints_.push_back(wp);
    }
    file.close();
}

void PurePursuitNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = msg;
    is_odom_received_ = true;
}

int PurePursuitNode::find_initial_target_point()
{
    if (!current_odom_ || waypoints_.empty()) {
        return -1;
    }

    double current_x = current_odom_->pose.pose.position.x;
    double current_y = current_odom_->pose.pose.position.y;
    
    // Find the closest point on the path
    double min_dist_sq = std::numeric_limits<double>::max();
    int closest_index = -1;

    for (size_t i = 0; i < waypoints_.size(); ++i) {
        double dx = waypoints_[i].x - current_x;
        double dy = waypoints_[i].y - current_y;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_index = i;
        }
    }

    // From the closest point, find the first point that is ahead of the lookahead distance
    // and is generally in front of the vehicle.
    if (closest_index != -1) {
        tf2::Quaternion q(
            current_odom_->pose.pose.orientation.x,
            current_odom_->pose.pose.orientation.y,
            current_odom_->pose.pose.orientation.z,
            current_odom_->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            int check_index = (closest_index + i) % waypoints_.size();
            double dist = std::hypot(waypoints_[check_index].x - current_x, waypoints_[check_index].y - current_y);
            
            if (dist > lookahead_distance_) {
                double path_yaw = std::atan2(waypoints_[check_index].y - current_y, waypoints_[check_index].x - current_x);
                double angle_diff = normalize_angle(path_yaw - current_yaw);

                if (std::abs(angle_diff) < M_PI_2) { // Check if the point is within a 90-degree arc in front
                    return check_index;
                }
            }
        }
    }

    return closest_index; // Fallback to closest point if no suitable lookahead found
}

void PurePursuitNode::control_loop()
{
    if (!is_odom_received_ || waypoints_.empty()) {
        return;
    }

    // --- Initialization ---
    if (target_index_ == -1) {
        target_index_ = find_initial_target_point();
        if (target_index_ != -1) {
            RCLCPP_INFO(this->get_logger(), "Initial target waypoint index set to: %d", target_index_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find initial target point. Waiting for odom.");
            return;
        }
    }

    // --- Pure Pursuit (Lateral Control) ---
    double current_x = current_odom_->pose.pose.position.x;
    double current_y = current_odom_->pose.pose.position.y;
    tf2::Quaternion q(
        current_odom_->pose.pose.orientation.x,
        current_odom_->pose.pose.orientation.y,
        current_odom_->pose.pose.orientation.z,
        current_odom_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    // Find the next target point based on lookahead distance
    // Search from the last target index to avoid going backwards
    bool found_target = false;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        int check_index = (target_index_ + i) % waypoints_.size();
        double dist = std::hypot(waypoints_[check_index].x - current_x, waypoints_[check_index].y - current_y);
        if (dist > lookahead_distance_) {
            target_index_ = check_index;
            found_target = true;
            break;
        }
    }
    
    // If we are at the end and no point is far enough, or simply passed the last point
    if (!found_target) { 
        target_index_ = (target_index_ + 1) % waypoints_.size();
        // If we wrapped around and the new target is too close, might need to re-evaluate
        // For infinite loop, just continue with the new target.
    }

    RCLCPP_INFO(this->get_logger(), "Current target index: %d", target_index_);

    const Waypoint &target_wp = waypoints_[target_index_];
    double target_x = target_wp.x;
    double target_y = target_wp.y;
    double target_v = target_wp.velocity;

    // Publish target marker for visualization
    publish_target_marker(target_x, target_y);

    // Transform target point to vehicle frame
    double tmp_x = target_x - current_x;
    double tmp_y = target_y - current_y;
    
    double rotated_x = tmp_x * std::cos(-current_yaw) - tmp_y * std::sin(-current_yaw);
    double rotated_y = tmp_x * std::sin(-current_yaw) + tmp_y * std::cos(-current_yaw);

    // Calculate steering angle (alpha)
    double alpha = std::atan2(rotated_y, rotated_x);
    double wheelbase = 1.04; // a parameter that should be tuned for the specific vehicle
    double steering_angle = std::atan((2 * wheelbase * std::sin(alpha)) / lookahead_distance_);
    
    // Invert steering angle if it's reversed in the simulator
    steering_angle = -steering_angle; 

    // Clamp steering angle to typical ROS range if necessary (e.g. -1.0 to 1.0)
    steering_angle = std::max(-1.0, std::min(1.0, steering_angle));

    // --- PID (Longitudinal Control) ---
    double current_velocity = std::hypot(current_odom_->twist.twist.linear.x, current_odom_->twist.twist.linear.y);
    double error = target_v - current_velocity;
    
    // Proportional
    double p_term = kp_ * error;
    
    // Integral
    pid_integral_ += error * (1.0 / loop_rate_);
    double i_term = ki_ * pid_integral_;
    
    // Derivative
    double derivative = (error - pid_previous_error_) * loop_rate_;
    double d_term = kd_ * derivative;
    pid_previous_error_ = error;
    
    // Output
    double control_output = p_term + i_term + d_term;
    
    double accel = 0.0;
    double brake = 0.0;
    if (control_output > 0) {
        accel = std::min(control_output, 1.0); // Clamp to [0, 1] for accel
    } else {
        brake = std::min(std::abs(control_output), 1.0); // Clamp to [0, 1] for brake
    }
            
    // --- Publish Control Command ---
    morai_msgs::msg::CtrlCmd ctrl_msg;
    ctrl_msg.longl_cmd_type = 1; // Throttle mode
    ctrl_msg.accel = static_cast<float>(accel);
    ctrl_msg.brake = static_cast<float>(brake);
    ctrl_msg.steering = static_cast<float>(steering_angle);
    control_pub_->publish(ctrl_msg);
}

void PurePursuitNode::publish_target_marker(double x, double y)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "pure_pursuit";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5; // slightly above ground
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    target_marker_pub_->publish(marker);
}

double PurePursuitNode::normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
