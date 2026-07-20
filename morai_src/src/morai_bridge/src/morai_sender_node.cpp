#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/ctrl_cmd.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::placeholders;

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
        this->declare_parameter<int>("cmd_udp_port", 9095);

        // 파라미터 값 가져오기
        simulator_ip_ = this->get_parameter("simulator_ip").as_string();
        cmd_udp_port_ = this->get_parameter("cmd_udp_port").as_int();

        // 제어 명령을 받는 서브스크라이버 생성
        subscription_ = this->create_subscription<morai_msgs::msg::CtrlCmd>(
            "/CtrlCmd", 10, std::bind(&MoraiSenderNode::ctrl_cmd_callback, this, _1));

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

    void ctrl_cmd_callback(const morai_msgs::msg::CtrlCmd::SharedPtr msg)
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
        cmd_packet.gear = 4;       // 4: Drive
        cmd_packet.cmd_type = 1;   // 1: Throttle
        cmd_packet.velocity = 0.0f;
        cmd_packet.acceleration = 0.0f;
        cmd_packet.accel = msg->accel;
        cmd_packet.brake = msg->brake;
        cmd_packet.steering = msg->steering;

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
    rclcpp::Subscription<morai_msgs::msg::CtrlCmd>::SharedPtr subscription_;

    // UDP 관련 멤버
    int send_sockfd_ = -1;
    struct sockaddr_in simulator_addr_;
    std::string simulator_ip_;
    int cmd_udp_port_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoraiSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
