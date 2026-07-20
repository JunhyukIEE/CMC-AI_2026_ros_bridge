#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<PurePursuitNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
