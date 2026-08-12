#ifndef UDP_CAMERA_RECEIVER_HPP_
#define UDP_CAMERA_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <nvjpeg.h>
#include <cuda_runtime.h>

namespace udp_camera_receiver
{

// MORAI UDP 패킷 헤더 구조체 (실제 구조 - 19 bytes)
#pragma pack(push, 1)
struct PacketHeader {
    char magic[3];              // 매직 넘버 "MOR" or "BOX" (3 bytes)
    uint8_t version;            // 버전/타입 (1 byte)
    uint8_t flags;              // 플래그 (1 byte)
    uint16_t unknown1;          // 알 수 없음 (2 bytes)
    uint32_t frame_id;          // 프레임 ID - 타임스탬프 (4 bytes)
    uint32_t packet_number;     // 패킷 번호 0부터 시작 (4 bytes)
    uint32_t payload_size;      // 이 패킷의 페이로드 크기 (4 bytes)
    // Total: 19 bytes
};
#pragma pack(pop)

// 카메라 설정 구조체
struct CameraConfig {
    std::string name;
    std::string ip;
    int port;
    std::string topic_name;
    int width;
    int height;
    int channels;
    double hfov_deg;
    bool compressed;
};

// 프레임 버퍼 구조체
struct FrameBuffer {
    uint32_t frame_number;
    uint32_t total_packets;
    uint32_t received_packets;
    std::map<uint32_t, std::vector<uint8_t>> received_payloads; // 패킷 번호 -> 페이로드
    std::vector<bool> packet_received;
    std::chrono::steady_clock::time_point last_update;
    bool is_jpeg;  // JPEG 압축 여부

    FrameBuffer() : frame_number(0), total_packets(0), received_packets(0), is_jpeg(false) {}
};

// 디코딩 작업 구조체
struct DecodeTask {
    int camera_index;
    uint32_t frame_id;
    bool is_jpeg;
    std::vector<uint8_t> data;
    rclcpp::Time stamp;
};

class UdpCameraReceiver : public rclcpp::Node
{
public:
    explicit UdpCameraReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~UdpCameraReceiver();

private:
    void loadParameters();
    void initializeCameras();
    void receiveThread(int camera_index);
    void processPacket(int camera_index, const uint8_t* data, size_t length);
    void synchronizerThread();
    void decodeWorkerThread(int worker_id);
    void initializeNvJpeg();
    void cleanupNvJpeg();
    bool decodeJpegGpu(int worker_id, const std::vector<uint8_t>& jpeg_data,
                       std::vector<uint8_t>& rgb_data, int& width, int& height);
    void queueDecodeTask(DecodeTask&& task);

    // 카메라 설정
    std::vector<CameraConfig> cameras_;

    // UDP 소켓
    std::vector<int> sockets_;

    // 퍼블리셔
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr>
        compressed_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;
    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs_;

    // 수신 스레드
    std::vector<std::shared_ptr<std::thread>> receive_threads_;
    std::shared_ptr<std::thread> synchronizer_thread_;
    std::atomic<bool> running_;

    // 프레임 버퍼
    std::vector<std::map<uint32_t, FrameBuffer>> frame_buffers_;
    std::vector<std::unique_ptr<std::mutex>> buffer_mutexes_;

    // 동기화 버퍼
    struct SyncFrame {
        int camera_index;
        uint32_t frame_id;
        bool is_jpeg;
        std::vector<uint8_t> data;
        std::chrono::steady_clock::time_point received_time;
    };
    std::map<uint32_t, std::vector<SyncFrame>> synchronized_frames_;
    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;

    // 디코딩 스레드풀
    static constexpr int kNumDecodeWorkers = 4;
    std::vector<std::shared_ptr<std::thread>> decode_workers_;
    std::queue<DecodeTask> decode_queue_;
    std::mutex decode_mutex_;
    std::condition_variable decode_cv_;

    // nvJPEG GPU 디코딩
    nvjpegHandle_t nvjpeg_handle_;
    std::vector<nvjpegJpegState_t> nvjpeg_states_;  // 워커당 하나
    std::vector<cudaStream_t> cuda_streams_;         // 워커당 하나
    bool nvjpeg_initialized_;

    // 파라미터
    bool debug_mode_;
    int max_buffered_frames_;
    double frame_timeout_sec_;
    bool publish_camera_info_;
    double default_hfov_deg_;
    bool enable_sync_;
    double sync_timeout_sec_;
    double sync_window_ms_;  // 동기화 윈도우 (밀리초) - 이 범위 내의 frame_id를 같은 프레임으로 취급

    // frame_id를 윈도우로 양자화
    uint32_t quantizeFrameId(uint32_t frame_id) const {
        uint32_t window_ns = static_cast<uint32_t>(sync_window_ms_ * 1000000.0);
        return (frame_id / window_ns) * window_ns;
    }
};

}  // namespace udp_camera_receiver

#endif  // UDP_CAMERA_RECEIVER_HPP_
