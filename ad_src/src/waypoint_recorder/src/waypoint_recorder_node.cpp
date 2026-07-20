#include "waypoint_recorder/waypoint_recorder.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<WaypointRecorder>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
