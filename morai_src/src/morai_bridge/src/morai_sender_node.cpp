#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::placeholders;

constexpr float MAX_STEERING_ANGLE_RAD = 40.0f * 3.14159265358979323846f / 180.0f;

constexpr float to_morai_steering(float steering_tire_angle)
{
    return std::clamp(
        -steering_tire_angle / MAX_STEERING_ANGLE_RAD, -1.0f, 1.0f);
}

constexpr int to_morai_gear(uint8_t gear)
{
    using Gear = autoware_vehicle_msgs::msg::GearCommand;
    if (gear >= Gear::DRIVE && gear <= Gear::DRIVE_18) return 4;
    if (gear == Gear::REVERSE || gear == Gear::REVERSE_2) return 2;
    if (gear == Gear::LOW || gear == Gear::LOW_2) return 5;
    if (gear == Gear::PARK) return 1;
    if (gear == Gear::NEUTRAL) return 3;
    if (gear == Gear::NONE) return 0;
    return -1;
}

static_assert(to_morai_gear(autoware_vehicle_msgs::msg::GearCommand::PARK) == 1);
static_assert(to_morai_gear(autoware_vehicle_msgs::msg::GearCommand::REVERSE) == 2);
static_assert(to_morai_gear(autoware_vehicle_msgs::msg::GearCommand::DRIVE) == 4);
static_assert(to_morai_steering(MAX_STEERING_ANGLE_RAD) == -1.0f);
static_assert(to_morai_steering(-MAX_STEERING_ANGLE_RAD) == 1.0f);

// MORAI CtrlCmd UDP 패킷 구조체
#pragma pack(push, 1)
struct CtrlCommandPacket {
    uint8_t mode;
    uint8_t gear;
    uint8_t cmd_type;
    float velocity;
    float acceleration;
    float accel;
    float brake;
    float steering;
};
#pragma pack(pop)

// UDP 헤더 구조체
#pragma pack(push, 1) // Added push for MoraiUDPHeader
struct MoraiUDPHeader {
    char start_char;
    char info_string[12]; // Corrected from 9 to 12
    char version_char;
    int32_t data_length;
    char aux_data[12];
};
#pragma pack(pop) // Added pop for MoraiUDPHeader


class MoraiSenderNode : public rclcpp::Node
{
public:
    MoraiSenderNode() : Node("morai_sender_node")
    {
        // ROS 파라미터 선언
        this->declare_parameter<std::string>("simulator_ip", "192.168.0.27");
        this->declare_parameter<int>("cmd_udp_port", 9091);

        // 파라미터 값 가져오기
        simulator_ip_ = this->get_parameter("simulator_ip").as_string();
        cmd_udp_port_ = this->get_parameter("cmd_udp_port").as_int();

        // 제어 명령을 받는 서브스크라이버 생성
        subscription_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1,
            std::bind(&MoraiSenderNode::ctrl_cmd_callback, this, _1));
        gear_subscription_ =
            this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
                "/control/command/gear_cmd", 10,
                std::bind(&MoraiSenderNode::gear_cmd_callback, this, _1));

        // UDP 소켓 초기화
        init_udp_socket();

        RCLCPP_INFO(this->get_logger(), "MoraiSenderNode has been started.");
        RCLCPP_INFO(this->get_logger(), "Will send CtrlCmd to %s:%d", simulator_ip_.c_str(), cmd_udp_port_);
    }

    ~MoraiSenderNode()
    {
        if (send_sockfd_ != -1) {
            close(send_sockfd_);
        }
    }

private:
    void gear_cmd_callback(
        const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
    {
        const int gear = to_morai_gear(msg->command);
        if (gear < 0) {
            RCLCPP_WARN(
                this->get_logger(), "Unsupported gear command: %u",
                static_cast<unsigned>(msg->command));
            return;
        }
        current_gear_ = static_cast<uint8_t>(gear);
    }

    void init_udp_socket()
    {
        send_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (send_sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed");
            return;
        }

        memset(&simulator_addr_, 0, sizeof(simulator_addr_));
        simulator_addr_.sin_family = AF_INET;
        simulator_addr_.sin_port = htons(cmd_udp_port_);
        if (inet_pton(AF_INET, simulator_ip_.c_str(), &simulator_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid simulator IP address");
            return;
        }
    } // <--- Added missing closing brace here

    void ctrl_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
    {
        if (send_sockfd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP socket is not ready.");
            return;
        }

        // 1. 헤더 채우기
        MoraiUDPHeader header;
        header.start_char = '#';
        memcpy(header.info_string, "MoraiCtrlCmd", 12); // Corrected length to 12
        header.version_char = '$';
        header.data_length = sizeof(CtrlCommandPacket);
        
        // aux_data: [1, 0, 0]
        int aux_values[] = {1, 0, 0};
        memcpy(header.aux_data, aux_values, sizeof(aux_values));

        // 2. 메시지 본문 채우기
        CtrlCommandPacket cmd_packet;
        cmd_packet.mode = 2;       // 2: AutoMode
        cmd_packet.gear = current_gear_;
        cmd_packet.cmd_type = msg->longitudinal.is_defined_acceleration ? 3 : 2;
        cmd_packet.velocity = msg->longitudinal.velocity * 3.6f;
        cmd_packet.acceleration = msg->longitudinal.acceleration;
        cmd_packet.accel = 0.0f;
        cmd_packet.brake = 0.0f;
        cmd_packet.steering = to_morai_steering(msg->lateral.steering_tire_angle);

        // 3. 전체 UDP 패킷 조립 (헤더 + 본문 + 테일)
        std::vector<unsigned char> packet_data;
        packet_data.resize(sizeof(header) + sizeof(cmd_packet) + 2);

        memcpy(packet_data.data(), &header, sizeof(header));
        memcpy(packet_data.data() + sizeof(header), &cmd_packet, sizeof(cmd_packet));
        
        // 테일 추가
        packet_data[sizeof(header) + sizeof(cmd_packet)] = '\r';
        packet_data[sizeof(header) + sizeof(cmd_packet) + 1] = '\n';

        // 4. UDP 패킷 전송
        sendto(send_sockfd_, packet_data.data(), packet_data.size(), 0,
               (const struct sockaddr *)&simulator_addr_, sizeof(simulator_addr_));
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Sent the first CtrlCmd packet to the simulator.");
    }

    // ROS 관련 멤버
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr subscription_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr
        gear_subscription_;

    // UDP 관련 멤버
    int send_sockfd_ = -1;
    struct sockaddr_in simulator_addr_;
    std::string simulator_ip_;
    int cmd_udp_port_;
    uint8_t current_gear_ = 4;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoraiSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
