#ifndef UDP_CAMERA_RECEIVER_HPP_
#define UDP_CAMERA_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
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
#include <array>
#include <cstdint>
#include <deque>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <nvjpeg.h>
#include <cuda_runtime.h>

namespace udp_camera_receiver
{

// MORAI camera UDP header fields. Wire bytes are decoded explicitly as little-endian.
struct PacketHeader {
    std::array<char, 3> magic;
    uint32_t total_second;
    uint32_t fraction;
    uint32_t packet_index;
    uint32_t payload_size;
};

struct BoxObject {
    std::array<float, 24> corners_3d;
    std::array<float, 4> bbox_2d_raw;
    std::string class_tag;
};

bool parseBoxPayload(
    const uint8_t* data, size_t size, std::vector<BoxObject>& objects,
    std::string& error);
std::string boxClassLabel(const std::string& raw_tag);

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
    uint64_t timestamp_ns;
    uint32_t total_packets;
    uint32_t received_packets;
    std::map<uint32_t, std::vector<uint8_t>> received_payloads; // 패킷 번호 -> 페이로드
    std::vector<bool> packet_received;
    std::chrono::steady_clock::time_point last_update;
    bool is_jpeg;  // JPEG 압축 여부

    FrameBuffer() : timestamp_ns(0), total_packets(0), received_packets(0), is_jpeg(false) {}
};

// 디코딩 작업 구조체
struct DecodeTask {
    int camera_index;
    uint64_t timestamp_ns;
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
    void publishBoxes(
        int camera_index, uint64_t box_timestamp_ns, uint64_t image_timestamp_ns,
        const std::vector<uint8_t>& data);
    void publishOverlayIfReady(int camera_index, uint64_t timestamp_ns);
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
    std::vector<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr>
        detection_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> overlay_publishers_;

    // 수신 스레드
    std::vector<std::shared_ptr<std::thread>> receive_threads_;
    std::shared_ptr<std::thread> synchronizer_thread_;
    std::atomic<bool> running_;

    // 프레임 버퍼
    std::vector<std::map<uint64_t, FrameBuffer>> frame_buffers_;
    std::vector<std::map<uint64_t, FrameBuffer>> box_buffers_;
    std::vector<std::deque<uint64_t>> recent_image_timestamps_;
    std::vector<std::unique_ptr<std::mutex>> buffer_mutexes_;

    // 동기화 버퍼
    struct SyncFrame {
        int camera_index;
        uint64_t timestamp_ns;
        bool is_jpeg;
        std::vector<uint8_t> data;
        std::chrono::steady_clock::time_point received_time;
    };
    std::map<uint64_t, std::vector<SyncFrame>> synchronized_frames_;
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
    bool publish_bbox_overlay_;
    int max_buffered_frames_;
    double frame_timeout_sec_;
    double bbox_match_tolerance_ms_;
    bool publish_camera_info_;
    double default_hfov_deg_;
    bool enable_sync_;
    double sync_timeout_sec_;
    double sync_window_ms_;

    std::vector<std::map<uint64_t, cv::Mat>> overlay_images_;
    std::vector<std::map<uint64_t, std::vector<BoxObject>>> overlay_boxes_;
    std::mutex overlay_mutex_;

    uint64_t quantizeTimestamp(uint64_t timestamp_ns) const {
        const uint64_t window_ns = static_cast<uint64_t>(sync_window_ms_ * 1000000.0);
        return window_ns ? (timestamp_ns / window_ns) * window_ns : timestamp_ns;
    }
};

}  // namespace udp_camera_receiver

#endif  // UDP_CAMERA_RECEIVER_HPP_
