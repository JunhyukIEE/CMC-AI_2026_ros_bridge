#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "morai_msgs/msg/set_traffic_light.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

// MORAI Set TrafficLight Ctrl UDP packet. The layout matches MORAI's official
// NetworkModule reference implementation (46 bytes, little-endian on x86).
#pragma pack(push, 1)
struct SetTrafficLightPacket
{
  std::array<char, 14> header;
  std::int32_t data_length;
  std::array<std::int32_t, 3> aux_data;
  std::array<char, 12> traffic_light_index;
  std::int16_t traffic_light_status;
  std::array<char, 2> tail;
};
#pragma pack(pop)

static_assert(sizeof(SetTrafficLightPacket) == 46, "Unexpected MORAI packet size");

constexpr std::int16_t kDefault = -1;
constexpr std::int16_t kValidStatusBits = 1 | 4 | 16 | 32;

bool is_valid_status(const std::int16_t status)
{
  return status == kDefault || (status > 0 && (status & ~kValidStatusBits) == 0);
}

}  // namespace

class MoraiTrafficLightSenderNode : public rclcpp::Node
{
public:
  MoraiTrafficLightSenderNode()
  : Node("morai_traffic_light_sender_node")
  {
    this->declare_parameter<std::string>("simulator_ip", "192.168.0.27");
    this->declare_parameter<int>("traffic_light_udp_port", 4000);
    this->declare_parameter<std::string>("input_topic", "/SetTrafficLight");

    simulator_ip_ = this->get_parameter("simulator_ip").as_string();
    traffic_light_udp_port_ = this->get_parameter("traffic_light_udp_port").as_int();
    input_topic_ = this->get_parameter("input_topic").as_string();

    if (!initialize_socket()) {
      throw std::runtime_error("Failed to initialize MORAI traffic-light UDP sender");
    }

    subscription_ = this->create_subscription<morai_msgs::msg::SetTrafficLight>(
      input_topic_, rclcpp::QoS(256),
      std::bind(&MoraiTrafficLightSenderNode::traffic_light_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Listening on %s; sending MORAI traffic-light control to %s:%d",
      input_topic_.c_str(), simulator_ip_.c_str(), traffic_light_udp_port_);
  }

  ~MoraiTrafficLightSenderNode() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  bool initialize_socket()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed: %s", std::strerror(errno));
      return false;
    }

    std::memset(&simulator_address_, 0, sizeof(simulator_address_));
    simulator_address_.sin_family = AF_INET;
    simulator_address_.sin_port = htons(static_cast<std::uint16_t>(traffic_light_udp_port_));
    if (inet_pton(AF_INET, simulator_ip_.c_str(), &simulator_address_.sin_addr) != 1) {
      RCLCPP_ERROR(this->get_logger(), "Invalid simulator IP address: %s", simulator_ip_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    return true;
  }

  void traffic_light_callback(const morai_msgs::msg::SetTrafficLight::SharedPtr message)
  {
    if (message->traffic_light_index.empty() || message->traffic_light_index.size() > 12) {
      RCLCPP_ERROR(
        this->get_logger(), "traffic_light_index must contain 1 to 12 characters; received '%s'",
        message->traffic_light_index.c_str());
      return;
    }
    if (!is_valid_status(message->traffic_light_status)) {
      RCLCPP_ERROR(
        this->get_logger(), "Unsupported traffic_light_status %d. Use -1 or a combination of 1, 4, 16, 32.",
        message->traffic_light_status);
      return;
    }

    SetTrafficLightPacket packet{};
    std::memcpy(packet.header.data(), "#TrafficLight$", packet.header.size());
    packet.data_length = 14;
    packet.aux_data = {0, 0, 0};
    packet.traffic_light_index.fill(' ');
    std::memcpy(
      packet.traffic_light_index.data(), message->traffic_light_index.data(),
      message->traffic_light_index.size());
    packet.traffic_light_status = message->traffic_light_status;
    packet.tail = {'\r', '\n'};

    const auto sent = sendto(
      socket_fd_, &packet, sizeof(packet), 0,
      reinterpret_cast<const sockaddr *>(&simulator_address_), sizeof(simulator_address_));
    if (sent != static_cast<ssize_t>(sizeof(packet))) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to send traffic-light command to %s:%d: %s", simulator_ip_.c_str(),
        traffic_light_udp_port_, std::strerror(errno));
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Sent traffic-light command: id='%s', status=%d", 
      message->traffic_light_index.c_str(), message->traffic_light_status);
  }

  int socket_fd_{-1};
  int traffic_light_udp_port_{};
  std::string simulator_ip_;
  std::string input_topic_;
  sockaddr_in simulator_address_{};
  rclcpp::Subscription<morai_msgs::msg::SetTrafficLight>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoraiTrafficLightSenderNode>());
  rclcpp::shutdown();
  return 0;
}
