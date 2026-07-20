#include "localization_converter/localization_converter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

LocalizationConverter::LocalizationConverter(const rclcpp::NodeOptions & options)
: Node("localization_converter_node", options)
{
  // Parameters
  this->declare_parameter<std::string>("ego_topic", "/Ego_topic");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("twist_topic", "/twist");
  this->declare_parameter<std::string>("pose_topic", "/pose");

  this->get_parameter("ego_topic", ego_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("twist_topic", twist_topic_);
  this->get_parameter("pose_topic", pose_topic_);

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscribers
  ego_sub_ = this->create_subscription<morai_msgs::msg::EgoVehicleStatus>(
    ego_topic_, 10, std::bind(&LocalizationConverter::ego_callback, this, _1));

  // Publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic_, 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
  
  RCLCPP_INFO(this->get_logger(), "Localization Converter Node has been started.");
  RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", ego_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s, %s, %s", odom_topic_.c_str(), twist_topic_.c_str(), pose_topic_.c_str());
}

void LocalizationConverter::ego_callback(const morai_msgs::msg::EgoVehicleStatus::SharedPtr msg)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->heading * M_PI / 180.0); // Heading is in degrees, convert to radians
  
  // TF broadcast
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg->position.x;
  t.transform.translation.y = msg->position.y;
  t.transform.translation.z = msg->position.z;
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
  
  // Odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header = t.header;
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = msg->position.x;
  odom_msg.pose.pose.position.y = msg->position.y;
  odom_msg.pose.pose.position.z = msg->position.z;
  odom_msg.pose.pose.orientation = tf2::toMsg(q);
  
  odom_msg.pose.covariance[0] = 0.1;   // x
  odom_msg.pose.covariance[7] = 0.1;   // y
  odom_msg.pose.covariance[14] = 0.1;  // z
  odom_msg.pose.covariance[21] = 0.05; // roll
  odom_msg.pose.covariance[28] = 0.05; // pitch
  odom_msg.pose.covariance[35] = 0.05; // yaw

  odom_msg.twist.twist.linear = msg->velocity;
  odom_msg.twist.covariance[0] = 0.1; // vx
  odom_msg.twist.covariance[7] = 0.1; // vy
  odom_msg.twist.covariance[35] = 0.1; // vyaw

  odom_pub_->publish(odom_msg);

  // Twist
  auto twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
  twist_msg.header = t.header;
  
  twist_msg.twist.twist.linear = msg->velocity;
  twist_msg.twist.covariance[0] = 0.1;
  twist_msg.twist.covariance[7] = 0.1;
  twist_msg.twist.covariance[14] = 0.1;

  twist_pub_->publish(twist_msg);

  // Pose
  auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg.header = t.header;

  pose_msg.pose.pose.position.x = msg->position.x;
  pose_msg.pose.pose.position.y = msg->position.y;
  pose_msg.pose.pose.position.z = msg->position.z;
  pose_msg.pose.pose.orientation = tf2::toMsg(q);

  pose_msg.pose.covariance[0] = 0.1;   // x
  pose_msg.pose.covariance[7] = 0.1;   // y
  pose_msg.pose.covariance[14] = 0.1;  // z
  pose_msg.pose.covariance[21] = 0.05; // roll
  pose_msg.pose.covariance[28] = 0.05; // pitch
  pose_msg.pose.covariance[35] = 0.05; // yaw

  pose_pub_->publish(pose_msg);
}
