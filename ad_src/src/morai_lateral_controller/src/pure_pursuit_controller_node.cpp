#include "morai_lateral_controller/pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<morai_lateral_controller::PurePursuitController>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
