#include "morai_longitudinal_controller/pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<morai_longitudinal_controller::PIDController>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
