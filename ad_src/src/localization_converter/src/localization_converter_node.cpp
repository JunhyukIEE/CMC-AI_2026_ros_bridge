#include "localization_converter/localization_converter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<LocalizationConverter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
