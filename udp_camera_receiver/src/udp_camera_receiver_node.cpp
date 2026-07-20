#include "udp_camera_receiver/udp_camera_receiver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<udp_camera_receiver::UdpCameraReceiver>(options);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
