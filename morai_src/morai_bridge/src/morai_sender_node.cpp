#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::placeholders;

constexpr float MAX_STEERING_ANGLE_RAD = 40.0f * 3.14159265358979323846f / 180.0f;
constexpr double MIN_CONTROLLER_DT_SEC = 0.001;
constexpr double MAX_CONTROLLER_DT_SEC = 0.1;
constexpr double STANDSTILL_TARGET_SPEED_MPS = 0.05;
constexpr double STANDSTILL_CURRENT_SPEED_MPS = 0.1;

struct PedalCommands {
    float throttle;
    float brake;
};

constexpr PedalCommands split_pedal_command(
    double output, double max_throttle, double max_brake)
{
    return {
        static_cast<float>(std::clamp(output, 0.0, max_throttle)),
        static_cast<float>(std::clamp(-output, 0.0, max_brake))};
}

constexpr float to_morai_steering(float steering_tire_angle)
{
    return std::clamp(
        steering_tire_angle / MAX_STEERING_ANGLE_RAD, -1.0f, 1.0f);
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
static_assert(to_morai_steering(MAX_STEERING_ANGLE_RAD) == 1.0f);
static_assert(to_morai_steering(-MAX_STEERING_ANGLE_RAD) == -1.0f);
static_assert(split_pedal_command(0.8, 0.5, 0.63).throttle == 0.5f);
static_assert(split_pedal_command(0.8, 0.5, 0.63).brake == 0.0f);
static_assert(split_pedal_command(-0.8, 0.5, 0.63).throttle == 0.0f);
static_assert(split_pedal_command(-0.8, 0.5, 0.63).brake == 0.63f);

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

        const auto nonnegative_parameter = [this](const char * name, double default_value) {
            this->declare_parameter<double>(name, default_value);
            const double value = this->get_parameter(name).as_double();
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
            }
            return value;
        };

        speed_kp_ = nonnegative_parameter("speed_kp", 0.05);
        speed_ki_ = nonnegative_parameter("speed_ki", 0.01);
        speed_kd_ = nonnegative_parameter("speed_kd", 0.0);
        integral_limit_ = nonnegative_parameter("integral_limit", 5.0);
        velocity_deadband_mps_ =
            nonnegative_parameter("velocity_deadband_mps", 0.1);
        max_throttle_command_ =
            nonnegative_parameter("max_throttle_command", 0.5);
        max_brake_command_ = nonnegative_parameter("max_brake_command", 0.63);
        standstill_brake_command_ =
            nonnegative_parameter("standstill_brake_command", 0.1);
        velocity_timeout_sec_ =
            nonnegative_parameter("velocity_timeout_sec", 0.2);

        // 파라미터 값 가져오기
        simulator_ip_ = this->get_parameter("simulator_ip").as_string();
        const auto cmd_udp_port = this->get_parameter("cmd_udp_port").as_int();
        if (cmd_udp_port < 1 || cmd_udp_port > 65535) {
            throw std::invalid_argument("cmd_udp_port must be between 1 and 65535");
        }
        cmd_udp_port_ = static_cast<int>(cmd_udp_port);
        if (max_throttle_command_ > 1.0 || max_brake_command_ > 1.0) {
            throw std::invalid_argument(
                "max_throttle_command and max_brake_command must not exceed 1.0");
        }
        if (standstill_brake_command_ > max_brake_command_) {
            throw std::invalid_argument(
                "standstill_brake_command must not exceed max_brake_command");
        }
        if (velocity_timeout_sec_ <= 0.0) {
            throw std::invalid_argument("velocity_timeout_sec must be greater than 0.0");
        }

        // 제어 명령을 받는 서브스크라이버 생성
        subscription_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1,
            std::bind(&MoraiSenderNode::ctrl_cmd_callback, this, _1));
        gear_subscription_ =
            this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
                "/control/command/gear_cmd", 10,
                std::bind(&MoraiSenderNode::gear_cmd_callback, this, _1));
        velocity_subscription_ =
            this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
                "/vehicle/status/velocity_status", 1,
                std::bind(&MoraiSenderNode::velocity_status_callback, this, _1));

        // UDP 소켓 초기화
        init_udp_socket();

        RCLCPP_INFO(this->get_logger(), "MoraiSenderNode has been started.");
        RCLCPP_INFO(
            this->get_logger(),
            "Will send CtrlCmd to %s:%d (speed PID %.3f/%.3f/%.3f, max throttle %.2f, max brake %.2f)",
            simulator_ip_.c_str(), cmd_udp_port_, speed_kp_, speed_ki_, speed_kd_,
            max_throttle_command_, max_brake_command_);
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
        const auto new_gear = static_cast<uint8_t>(gear);
        if (new_gear != current_gear_) {
            reset_speed_controller();
            current_gear_ = new_gear;
        }
    }

    void velocity_status_callback(
        const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        if (!std::isfinite(msg->longitudinal_velocity)) {
            has_current_velocity_ = false;
            reset_speed_controller();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Ignoring non-finite vehicle velocity.");
            return;
        }

        current_velocity_mps_ = msg->longitudinal_velocity;
        last_velocity_receive_time_ = std::chrono::steady_clock::now();
        has_current_velocity_ = true;
    }

    void reset_speed_controller()
    {
        integral_error_ = 0.0;
        previous_measured_speed_mps_ = 0.0;
        has_previous_control_sample_ = false;
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
            close(send_sockfd_);
            send_sockfd_ = -1;
            throw std::invalid_argument("simulator_ip must be a valid IPv4 address");
        }
    } // <--- Added missing closing brace here

    void ctrl_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
    {
        if (send_sockfd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP socket is not ready.");
            return;
        }

        const double target_velocity = msg->longitudinal.velocity;
        const float steering = msg->lateral.steering_tire_angle;
        const auto now = std::chrono::steady_clock::now();
        PedalCommands pedals{0.0f, 0.0f};
        const bool control_valid =
            std::isfinite(steering) && std::isfinite(target_velocity);
        if (!control_valid) {
            reset_speed_controller();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Non-finite control command; sending zero throttle and brake.");
        } else if (!has_current_velocity_ ||
            std::chrono::duration<double>(now - last_velocity_receive_time_).count() >
                velocity_timeout_sec_) {
            reset_speed_controller();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Vehicle velocity is unavailable or stale; sending zero throttle and brake.");
        } else {
            const double target_speed = std::abs(target_velocity);
            const double measured_speed = std::abs(current_velocity_mps_);

            if (target_speed <= STANDSTILL_TARGET_SPEED_MPS &&
                measured_speed <= STANDSTILL_CURRENT_SPEED_MPS) {
                pedals.brake = static_cast<float>(standstill_brake_command_);
                reset_speed_controller();
            } else {
                double error = target_speed - measured_speed;
                if (std::abs(error) <= velocity_deadband_mps_) {
                    error = 0.0;
                }

                double derivative = 0.0;
                double candidate_integral = integral_error_;
                if (has_previous_control_sample_) {
                    const double dt = std::clamp(
                        std::chrono::duration<double>(now - previous_control_time_).count(),
                        MIN_CONTROLLER_DT_SEC, MAX_CONTROLLER_DT_SEC);
                    candidate_integral = std::clamp(
                        integral_error_ + error * dt, -integral_limit_, integral_limit_);
                    derivative = -speed_kd_ *
                        (measured_speed - previous_measured_speed_mps_) / dt;
                }

                const double candidate_output =
                    speed_kp_ * error + speed_ki_ * candidate_integral + derivative;
                const bool saturating_throttle =
                    candidate_output > max_throttle_command_ && error > 0.0;
                const bool saturating_brake =
                    candidate_output < -max_brake_command_ && error < 0.0;
                if (!saturating_throttle && !saturating_brake) {
                    integral_error_ = candidate_integral;
                }

                const double output =
                    speed_kp_ * error + speed_ki_ * integral_error_ + derivative;
                pedals = split_pedal_command(
                    output, max_throttle_command_, max_brake_command_);
                previous_control_time_ = now;
                previous_measured_speed_mps_ = measured_speed;
                has_previous_control_sample_ = true;
            }
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
        // Pedal mode keeps throttle and brake under this node's limits.
        cmd_packet.cmd_type = 1;
        cmd_packet.velocity = 0.0f;
        cmd_packet.acceleration = 0.0f;
        cmd_packet.accel = pedals.throttle;
        cmd_packet.brake = pedals.brake;
        cmd_packet.steering =
            control_valid ? to_morai_steering(steering) : 0.0f;

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
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
        velocity_subscription_;

    // UDP 관련 멤버
    int send_sockfd_ = -1;
    struct sockaddr_in simulator_addr_;
    std::string simulator_ip_;
    int cmd_udp_port_;
    double speed_kp_ = 0.05;
    double speed_ki_ = 0.01;
    double speed_kd_ = 0.0;
    double integral_limit_ = 5.0;
    double velocity_deadband_mps_ = 0.1;
    double max_throttle_command_ = 0.5;
    double max_brake_command_ = 0.63;
    double standstill_brake_command_ = 0.1;
    double velocity_timeout_sec_ = 0.2;
    double current_velocity_mps_ = 0.0;
    double integral_error_ = 0.0;
    double previous_measured_speed_mps_ = 0.0;
    std::chrono::steady_clock::time_point last_velocity_receive_time_;
    std::chrono::steady_clock::time_point previous_control_time_;
    bool has_current_velocity_ = false;
    bool has_previous_control_sample_ = false;
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
