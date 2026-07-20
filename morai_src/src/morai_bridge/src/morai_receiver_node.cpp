#include "rclcpp/rclcpp.hpp"
#include "morai_msgs/msg/ego_vehicle_status.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>

// MORAI EgoVehicleStatus UDP 패킷 구조체
// #pragma pack(push, 1)을 사용하여 C++ 컴파일러가 멤버 사이에 패딩 바이트를 추가하는 것을 방지합니다.
// 이렇게 하면 UDP 패킷의 메모리 레이아웃과 정확히 일치하게 됩니다.
#pragma pack(push, 1)
struct EgoVehicleStatusUDP {
    int32_t time_stamp_sec;
    int32_t time_stamp_nanosec;
    int8_t ctrl_mode;
    int8_t gear;
    float signed_velocity;
    int32_t map_data_id;
    float accel;
    float brake;
    float size_x;
    float size_y;
    float size_z;
    float overhang;
    float wheelbase;
    float rear_overhang;
    float pos_x;
    float pos_y;
    float pos_z;
    float roll;
    float pitch;
    float yaw;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float steer;
    char link_id[38];
};
#pragma pack(pop)

// UDP 헤더 구조체
#pragma pack(push, 1)
struct MoraiUDPHeader {
    char start_char;      // '#'
    char info_string[9];  // "Morailnfo"
    char version_char;    // '$'
    int32_t data_length;
    char aux_data[12];
};
#pragma pack(pop)


class MoraiReceiverNode : public rclcpp::Node
{
public:
    MoraiReceiverNode() : Node("morai_receiver_node")
    {
        // 파라미터 선언 (UDP 포트, odom 프레임 ID)
        this->declare_parameter<int>("udp_port", 9000);
        this->declare_parameter<std::string>("frame_id", "map");


        // 파라미터 값 가져오기
        udp_port_ = this->get_parameter("udp_port").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        
        // EgoVehicleStatus 정보를 발행할 퍼블리셔 생성
        ego_topic_pub_ = this->create_publisher<morai_msgs::msg::EgoVehicleStatus>("/Ego_topic", 10);

        RCLCPP_INFO(this->get_logger(), "MoraiBridgeNode has been started.");
        RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port: %d", udp_port_);

        // 별도의 스레드에서 UDP 리스너 실행
        udp_thread_ = std::thread(&MoraiReceiverNode::udp_listener, this);
    }

    ~MoraiReceiverNode()
    {
        // 노드 종료 시 스레드가 안전하게 종료되도록 함
        if (udp_thread_.joinable()) {
            udp_thread_.join();
        }
    }

private:
    void udp_listener()
    {
        int sockfd;
        struct sockaddr_in servaddr, cliaddr;

        // 소켓 파일 디스크립터 생성
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed");
            return;
        }

        // 서버 주소 구조체 초기화
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET; // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY; // 모든 IP 주소에서 수신
        servaddr.sin_port = htons(udp_port_); // 설정된 포트

        // 소켓에 서버 주소 바인딩
        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP bind failed on port %d", udp_port_);
            close(sockfd);
            return;
        }

        char buffer[2048]; // UDP 패킷을 수신할 버퍼

        while (rclcpp::ok()) {
            socklen_t len = sizeof(cliaddr);
            // blocking call, UDP 데이터가 들어올 때까지 대기
            ssize_t n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);

            if (n < 0) {
                RCLCPP_WARN(this->get_logger(), "recvfrom error");
                continue;
            }

            // 패킷 크기 검사 (헤더 + 데이터)
            if (n < (sizeof(MoraiUDPHeader) + sizeof(EgoVehicleStatusUDP))) {
                RCLCPP_WARN(this->get_logger(), "Received a packet smaller than expected size. Got %ld bytes.", n);
                continue;
            }

            // 버퍼에서 헤더와 데이터를 파싱
            MoraiUDPHeader* header = reinterpret_cast<MoraiUDPHeader*>(buffer);
            EgoVehicleStatusUDP* status = reinterpret_cast<EgoVehicleStatusUDP*>(buffer + sizeof(MoraiUDPHeader));

            // 헤더 유효성 검사 (옵션)
            if (header->start_char != '#' || header->version_char != '$') {
                RCLCPP_WARN(this->get_logger(), "Received packet with invalid header.");
                continue;
            }
            
            // 파싱된 데이터를 EgoVehicleStatus 메시지로 변환하고 발행
            publish_ego_vehicle_status(status);
        }

        close(sockfd);
    }

    void publish_ego_vehicle_status(const EgoVehicleStatusUDP* status)
    {
        auto ego_msg = std::make_unique<morai_msgs::msg::EgoVehicleStatus>();

        // 1. Header 설정
        ego_msg->header.stamp.sec = status->time_stamp_sec;
        ego_msg->header.stamp.nanosec = status->time_stamp_nanosec;
        ego_msg->header.frame_id = frame_id_;
        
        ego_msg->unique_id = 0; // or some unique id

        // 2. Kinematics
        ego_msg->position.x = status->pos_x;
        ego_msg->position.y = status->pos_y;
        ego_msg->position.z = status->pos_z;

        ego_msg->acceleration.x = status->accel_x;
        ego_msg->acceleration.y = status->accel_y;
        ego_msg->acceleration.z = status->accel_z;

        // MORAI의 속도(km/h)를 ROS의 속도(m/s)로 변환 (1 km/h = 0.277778 m/s)
        ego_msg->velocity.x = status->velocity_x * 0.277778;
        ego_msg->velocity.y = status->velocity_y * 0.277778;
        ego_msg->velocity.z = status->velocity_z * 0.277778;
        
        ego_msg->heading = status->yaw;
        ego_msg->accel = status->accel;
        ego_msg->brake = status->brake;
        ego_msg->wheel_angle = status->steer;


        ego_topic_pub_->publish(std::move(ego_msg));
    }

    // ROS 관련 멤버
    rclcpp::Publisher<morai_msgs::msg::EgoVehicleStatus>::SharedPtr ego_topic_pub_;

    // UDP 관련 멤버
    std::thread udp_thread_;
    int udp_port_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoraiReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
