#ifndef LOCALIZATION_CONVERTER__LOCALIZATION_CONVERTER_HPP_
#define LOCALIZATION_CONVERTER__LOCALIZATION_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/ego_vehicle_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class LocalizationConverter : public rclcpp::Node
{
public:
  explicit LocalizationConverter(const rclcpp::NodeOptions & options);

private:
  void ego_callback(const morai_msgs::msg::EgoVehicleStatus::SharedPtr msg);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<morai_msgs::msg::EgoVehicleStatus>::SharedPtr ego_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  
  // Parameters
  std::string ego_topic_;
  std::string odom_topic_;
  std::string twist_topic_;
  std::string pose_topic_;
};

#endif  // LOCALIZATION_CONVERTER__LOCALIZATION_CONVERTER_HPP_
