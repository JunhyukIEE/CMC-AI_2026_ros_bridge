#include "udp_camera_receiver/udp_camera_receiver.hpp"
#include <cstring>
#include <cmath>
#include <utility>
#include <algorithm>
#include <cctype>
#include <limits>
#include <iomanip>
#include <sstream>

namespace udp_camera_receiver
{

namespace
{
constexpr size_t kHeaderSize = 19;
constexpr size_t kTailSize = 2;
constexpr size_t kBoxObjectSize = 115;

uint32_t readLe32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24);
}

float readLeFloat(const uint8_t* data)
{
    const uint32_t bits = readLe32(data);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool parseHeader(const uint8_t* data, size_t length, PacketHeader& header)
{
    if (length < kHeaderSize + kTailSize) return false;
    std::copy_n(reinterpret_cast<const char*>(data), 3, header.magic.begin());
    header.total_second = readLe32(data + 3);
    header.fraction = readLe32(data + 7);
    header.packet_index = readLe32(data + 11);
    header.payload_size = readLe32(data + 15);
    // MORAI pads UDP datagrams to 65,000 bytes; Size is the useful payload length.
    return header.payload_size <= length - kHeaderSize - kTailSize;
}

bool magicIs(const PacketHeader& header, const char* magic)
{
    return std::equal(header.magic.begin(), header.magic.end(), magic);
}

bool timestampNanoseconds(const PacketHeader& header, bool is_box, uint64_t& timestamp_ns)
{
    (void)is_box;
    // Actual 24.R2 BOX packets use nanoseconds despite the web page saying milliseconds.
    if (header.fraction >= 1000000000U) {
        return false;
    }
    timestamp_ns = header.total_second * 1000000000ULL + header.fraction;
    return timestamp_ns <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
}

std::string imageTopic(std::string topic, bool compressed)
{
    const std::string suffix = "/compressed";
    const bool has_suffix = topic.size() >= suffix.size() &&
        topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0;
    if (compressed && !has_suffix) {
        topic += suffix;
    } else if (!compressed && has_suffix) {
        topic.erase(topic.size() - suffix.size());
    }
    return topic;
}

std::string cameraBaseTopic(std::string topic)
{
    const auto pos = topic.find("/image_raw");
    if (pos != std::string::npos) topic.erase(pos);
    return topic;
}
}  // namespace

bool parseBoxPayload(
    const uint8_t* data, size_t size, std::vector<BoxObject>& objects,
    std::string& error)
{
    objects.clear();
    if (size % kBoxObjectSize != 0) {
        error = "payload size is not a multiple of 115";
        return false;
    }

    objects.reserve(size / kBoxObjectSize);
    for (size_t offset = 0; offset < size; offset += kBoxObjectSize) {
        BoxObject object;
        for (size_t i = 0; i < object.corners_3d.size(); ++i) {
            object.corners_3d[i] = readLeFloat(data + offset + i * sizeof(float));
            if (!std::isfinite(object.corners_3d[i])) {
                error = "3D bbox contains NaN/Inf";
                return false;
            }
        }
        for (size_t i = 0; i < object.bbox_2d_raw.size(); ++i) {
            object.bbox_2d_raw[i] = readLeFloat(data + offset + 96 + i * sizeof(float));
            if (!std::isfinite(object.bbox_2d_raw[i])) {
                error = "2D bbox contains NaN/Inf";
                return false;
            }
        }
        object.class_tag.assign(reinterpret_cast<const char*>(data + offset + 112), 3);
        object.class_tag.erase(
            std::remove_if(object.class_tag.begin(), object.class_tag.end(), [](unsigned char c) {
                return c == '\0' || std::isspace(c);
            }), object.class_tag.end());
        objects.push_back(std::move(object));
    }
    return true;
}

std::string boxClassLabel(const std::string& raw_tag)
{
    if (raw_tag == std::string("\xff\x02\x02", 3)) return "Vehicle";
    if (raw_tag == std::string("\x62\x02\xff", 3)) return "Pedestrian";
    if (raw_tag == std::string("\xec\xff\x02", 3)) return "Obstacle";
    return "Unknown";
}

UdpCameraReceiver::UdpCameraReceiver(const rclcpp::NodeOptions & options)
: Node("udp_camera_receiver", options), running_(true), nvjpeg_initialized_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing UDP Camera Receiver...");

    loadParameters();
    initializeCameras();
    initializeNvJpeg();

    // 디코딩 스레드풀 시작
    for (int i = 0; i < kNumDecodeWorkers; ++i) {
        decode_workers_.push_back(std::make_shared<std::thread>(
            &UdpCameraReceiver::decodeWorkerThread, this, i));
    }
    RCLCPP_INFO(this->get_logger(), "Started %d decode worker threads", kNumDecodeWorkers);

    // 동기화 스레드 시작
    if (enable_sync_) {
        synchronizer_thread_ = std::make_shared<std::thread>(
            &UdpCameraReceiver::synchronizerThread, this);
        RCLCPP_INFO(this->get_logger(), "Synchronization enabled. Starting synchronizer thread.");
    }

    RCLCPP_INFO(this->get_logger(), "UDP Camera Receiver initialized successfully");
}

UdpCameraReceiver::~UdpCameraReceiver()
{
    running_ = false;
    sync_cv_.notify_all();
    decode_cv_.notify_all();

    // 동기화 스레드 종료
    if (synchronizer_thread_ && synchronizer_thread_->joinable()) {
        synchronizer_thread_->join();
    }

    // 디코딩 스레드풀 종료
    for (auto& worker : decode_workers_) {
        if (worker && worker->joinable()) {
            worker->join();
        }
    }

    // 스레드 종료 대기
    for (auto& thread : receive_threads_) {
        if (thread && thread->joinable()) {
            thread->join();
        }
    }

    // 소켓 닫기
    for (auto socket : sockets_) {
        if (socket >= 0) {
            close(socket);
        }
    }

    // nvJPEG 정리
    cleanupNvJpeg();

    RCLCPP_INFO(this->get_logger(), "UDP Camera Receiver shut down");
}

void UdpCameraReceiver::loadParameters()
{
    // 일반 파라미터
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<bool>("publish_bbox_overlay", false);
    this->declare_parameter<int>("max_buffered_frames", 3);
    this->declare_parameter<double>("frame_timeout_sec", 1.0);
    this->declare_parameter<double>("bbox_match_tolerance_ms", 20.0);
    this->declare_parameter<bool>("publish_camera_info", true);
    this->declare_parameter<double>("hfov_deg", 90.0);
    this->declare_parameter<bool>("enable_sync", false);
    this->declare_parameter<double>("sync_timeout_sec", 0.1);
    this->declare_parameter<double>("sync_window_ms", 30.0);  // 기본 30ms 윈도우

    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("publish_bbox_overlay", publish_bbox_overlay_);
    this->get_parameter("max_buffered_frames", max_buffered_frames_);
    this->get_parameter("frame_timeout_sec", frame_timeout_sec_);
    this->get_parameter("bbox_match_tolerance_ms", bbox_match_tolerance_ms_);
    this->get_parameter("publish_camera_info", publish_camera_info_);
    default_hfov_deg_ = this->get_parameter("hfov_deg").as_double();
    this->get_parameter("enable_sync", enable_sync_);
    this->get_parameter("sync_timeout_sec", sync_timeout_sec_);
    this->get_parameter("sync_window_ms", sync_window_ms_);

    RCLCPP_INFO(this->get_logger(), "Sync window: %.1f ms", sync_window_ms_);

    // 카메라 개수
    this->declare_parameter<int>("num_cameras", 1);
    int num_cameras = this->get_parameter("num_cameras").as_int();

    RCLCPP_INFO(this->get_logger(), "Number of cameras: %d", num_cameras);

    // 각 카메라 설정 로드
    for (int i = 0; i < num_cameras; ++i) {
        CameraConfig config;

        std::string prefix = "camera_" + std::to_string(i) + ".";

        this->declare_parameter<std::string>(prefix + "name", "camera_" + std::to_string(i));
        this->declare_parameter<std::string>(prefix + "ip", "192.168.0.37");
        this->declare_parameter<int>(prefix + "port", 9001 + i);
        this->declare_parameter<std::string>(prefix + "topic_name", "/camera_" + std::to_string(i) + "/image_raw");
        this->declare_parameter<int>(prefix + "width", 640);
        this->declare_parameter<int>(prefix + "height", 480);
        this->declare_parameter<int>(prefix + "channels", 3);
        this->declare_parameter<double>(prefix + "hfov_deg", default_hfov_deg_);
        this->declare_parameter<bool>(prefix + "compressed", false);

        config.name = this->get_parameter(prefix + "name").as_string();
        config.ip = this->get_parameter(prefix + "ip").as_string();
        config.port = this->get_parameter(prefix + "port").as_int();
        config.topic_name = this->get_parameter(prefix + "topic_name").as_string();
        config.width = this->get_parameter(prefix + "width").as_int();
        config.height = this->get_parameter(prefix + "height").as_int();
        config.channels = this->get_parameter(prefix + "channels").as_int();
        config.hfov_deg = this->get_parameter(prefix + "hfov_deg").as_double();
        config.compressed = this->get_parameter(prefix + "compressed").as_bool();
        config.topic_name = imageTopic(config.topic_name, config.compressed);

        cameras_.push_back(config);

        RCLCPP_INFO(this->get_logger(), "Camera %d: %s - %s:%d -> %s (%dx%d)",
                    i, config.name.c_str(), config.ip.c_str(), config.port,
                    config.topic_name.c_str(), config.width, config.height);
    }
}

void UdpCameraReceiver::initializeCameras()
{
    const auto camera_qos = rclcpp::SensorDataQoS().keep_last(1);
    for (size_t i = 0; i < cameras_.size(); ++i) {
        const auto& config = cameras_[i];

        constexpr double kPi = 3.14159265358979323846;

        // UDP 소켓 생성
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket for camera %zu", i);
            sockets_.push_back(-1);
            continue;
        }

        // 소켓 옵션 설정 (수신 버퍼 크기 증가)
        int buffer_size = 1024 * 1024;  // 1MB
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));

        // 소켓 타임아웃 설정 (0.5초) - 스레드 종료를 위해 필요
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // 주소 설정
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.port);
        addr.sin_addr.s_addr = inet_addr(config.ip.c_str());

        // 바인딩
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket for camera %zu at %s:%d - %s",
                        i, config.ip.c_str(), config.port, strerror(errno));
            close(sock);
            sockets_.push_back(-1);
            continue;
        }

        sockets_.push_back(sock);

        // 퍼블리셔 생성
        if (config.compressed) {
            publishers_.push_back(nullptr);
            compressed_publishers_.push_back(
                this->create_publisher<sensor_msgs::msg::CompressedImage>(
                    config.topic_name, camera_qos));
        } else {
            publishers_.push_back(
                this->create_publisher<sensor_msgs::msg::Image>(
                    config.topic_name, camera_qos));
            compressed_publishers_.push_back(nullptr);
        }

        const std::string base_topic = cameraBaseTopic(config.topic_name);
        detection_publishers_.push_back(
            this->create_publisher<vision_msgs::msg::Detection2DArray>(
                base_topic + "/ground_truth/detections", camera_qos));
        if (publish_bbox_overlay_) {
            overlay_publishers_.push_back(
                this->create_publisher<sensor_msgs::msg::Image>(
                    base_topic + "/ground_truth/debug_overlay", camera_qos));
        } else {
            overlay_publishers_.push_back(nullptr);
        }

        if (publish_camera_info_) {
            std::string info_topic = config.topic_name;
            const std::string compressed_suffix = "/image_raw/compressed";
            const std::string suffix = "/image_raw";
            if (info_topic.size() >= compressed_suffix.size() &&
                info_topic.compare(
                    info_topic.size() - compressed_suffix.size(),
                    compressed_suffix.size(), compressed_suffix) == 0) {
                info_topic =
                    info_topic.substr(0, info_topic.size() - compressed_suffix.size()) +
                    "/camera_info";
            } else if (info_topic.size() >= suffix.size() &&
                info_topic.compare(info_topic.size() - suffix.size(), suffix.size(), suffix) == 0) {
                info_topic = info_topic.substr(0, info_topic.size() - suffix.size()) + "/camera_info";
            } else {
                info_topic += "/camera_info";
            }

            auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                info_topic, camera_qos);
            camera_info_publishers_.push_back(camera_info_pub);

            sensor_msgs::msg::CameraInfo info_msg;
            info_msg.header.frame_id = config.name;
            info_msg.width = config.width;
            info_msg.height = config.height;
            info_msg.distortion_model = "plumb_bob";
            info_msg.d.assign(5, 0.0);

            double hfov_rad = config.hfov_deg * kPi / 180.0;
            double fx = static_cast<double>(config.width) / (2.0 * std::tan(hfov_rad / 2.0));
            double vfov_rad = 2.0 * std::atan(
                static_cast<double>(config.height) / static_cast<double>(config.width) * std::tan(hfov_rad / 2.0));
            double fy = static_cast<double>(config.height) / (2.0 * std::tan(vfov_rad / 2.0));
            double cx = static_cast<double>(config.width) / 2.0;
            double cy = static_cast<double>(config.height) / 2.0;

            info_msg.k = {fx, 0.0, cx,
                          0.0, fy, cy,
                          0.0, 0.0, 1.0};

            info_msg.r = {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};

            info_msg.p = {fx, 0.0, cx, 0.0,
                          0.0, fy, cy, 0.0,
                          0.0, 0.0, 1.0, 0.0};

            camera_info_msgs_.push_back(info_msg);

            RCLCPP_INFO(this->get_logger(),
                        "Camera %zu CameraInfo: %s (hfov=%.1f, fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f)",
                        i, info_topic.c_str(), config.hfov_deg, fx, fy, cx, cy);
        } else {
            camera_info_publishers_.push_back(nullptr);
            camera_info_msgs_.emplace_back();
        }

        // 프레임 버퍼 초기화
        frame_buffers_.emplace_back();
        box_buffers_.emplace_back();
        recent_image_timestamps_.emplace_back();
        overlay_images_.emplace_back();
        overlay_boxes_.emplace_back();
        buffer_mutexes_.push_back(std::make_unique<std::mutex>());

        // 수신 스레드 시작
        auto thread = std::make_shared<std::thread>(
            &UdpCameraReceiver::receiveThread, this, i);
        receive_threads_.push_back(thread);

        RCLCPP_INFO(this->get_logger(), "Camera %zu initialized: listening on %s:%d",
                    i, config.ip.c_str(), config.port);
    }
}

void UdpCameraReceiver::initializeNvJpeg()
{
    nvjpegStatus_t status = nvjpegCreateSimple(&nvjpeg_handle_);
    if (status != NVJPEG_STATUS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create nvJPEG handle: %d", status);
        return;
    }

    nvjpeg_states_.resize(kNumDecodeWorkers);
    cuda_streams_.resize(kNumDecodeWorkers);

    for (int i = 0; i < kNumDecodeWorkers; ++i) {
        status = nvjpegJpegStateCreate(nvjpeg_handle_, &nvjpeg_states_[i]);
        if (status != NVJPEG_STATUS_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create nvJPEG state %d: %d", i, status);
            return;
        }

        cudaError_t cuda_status = cudaStreamCreate(&cuda_streams_[i]);
        if (cuda_status != cudaSuccess) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CUDA stream %d: %s",
                        i, cudaGetErrorString(cuda_status));
            return;
        }
    }

    nvjpeg_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "nvJPEG GPU decoder initialized with %d workers", kNumDecodeWorkers);
}

void UdpCameraReceiver::cleanupNvJpeg()
{
    if (!nvjpeg_initialized_) return;

    for (int i = 0; i < kNumDecodeWorkers; ++i) {
        if (i < static_cast<int>(nvjpeg_states_.size())) {
            nvjpegJpegStateDestroy(nvjpeg_states_[i]);
        }
        if (i < static_cast<int>(cuda_streams_.size())) {
            cudaStreamDestroy(cuda_streams_[i]);
        }
    }
    nvjpegDestroy(nvjpeg_handle_);
    nvjpeg_initialized_ = false;
}

bool UdpCameraReceiver::decodeJpegGpu(int worker_id, const std::vector<uint8_t>& jpeg_data,
                                       std::vector<uint8_t>& rgb_data, int& width, int& height)
{
    if (!nvjpeg_initialized_ || worker_id >= kNumDecodeWorkers) {
        return false;
    }

    // JPEG 정보 읽기
    int nComponents;
    nvjpegChromaSubsampling_t subsampling;
    int widths[NVJPEG_MAX_COMPONENT];
    int heights[NVJPEG_MAX_COMPONENT];

    nvjpegStatus_t status = nvjpegGetImageInfo(
        nvjpeg_handle_, jpeg_data.data(), jpeg_data.size(),
        &nComponents, &subsampling, widths, heights);

    if (status != NVJPEG_STATUS_SUCCESS) {
        return false;
    }

    width = widths[0];
    height = heights[0];

    // GPU 메모리 할당
    nvjpegImage_t output_image;
    for (int c = 0; c < NVJPEG_MAX_COMPONENT; c++) {
        output_image.channel[c] = nullptr;
        output_image.pitch[c] = 0;
    }

    // RGB interleaved 출력 (3채널)
    size_t pitch = width * 3;
    unsigned char* d_output = nullptr;
    cudaError_t cuda_status = cudaMalloc(&d_output, pitch * height);
    if (cuda_status != cudaSuccess) {
        return false;
    }

    output_image.channel[0] = d_output;
    output_image.pitch[0] = pitch;

    // 디코딩 (RGB interleaved)
    status = nvjpegDecode(
        nvjpeg_handle_, nvjpeg_states_[worker_id],
        jpeg_data.data(), jpeg_data.size(),
        NVJPEG_OUTPUT_RGBI, &output_image,
        cuda_streams_[worker_id]);

    if (status != NVJPEG_STATUS_SUCCESS) {
        cudaFree(d_output);
        return false;
    }

    // 스트림 동기화
    cudaStreamSynchronize(cuda_streams_[worker_id]);

    // GPU -> CPU 복사
    rgb_data.resize(pitch * height);
    cuda_status = cudaMemcpy(rgb_data.data(), d_output, pitch * height, cudaMemcpyDeviceToHost);
    cudaFree(d_output);

    return cuda_status == cudaSuccess;
}

void UdpCameraReceiver::receiveThread(int camera_index)
{
    if (camera_index >= static_cast<int>(sockets_.size()) || sockets_[camera_index] < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid socket for camera %d", camera_index);
        return;
    }

    int sock = sockets_[camera_index];
    const size_t buffer_size = 65536;  // 64KB 버퍼
    std::vector<uint8_t> buffer(buffer_size);

    RCLCPP_INFO(this->get_logger(), "Receive thread started for camera %d", camera_index);

    while (running_ && rclcpp::ok()) {
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t received = recvfrom(sock, buffer.data(), buffer_size, 0,
                                    (struct sockaddr*)&sender_addr, &sender_len);

        if (received > 0) {
            processPacket(camera_index, buffer.data(), received);
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "Error receiving data for camera %d: %s",
                        camera_index, strerror(errno));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Receive thread stopped for camera %d", camera_index);
}

void UdpCameraReceiver::processPacket(int camera_index, const uint8_t* data, size_t length)
{
    PacketHeader header;
    if (!parseHeader(data, length, header)) {
        if (length >= kHeaderSize) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Cam %d: invalid %.3s packet length=%zu sec=%u fraction=%u index=%u size=%u",
                camera_index, reinterpret_cast<const char*>(data), length, readLe32(data + 3),
                readLe32(data + 7), readLe32(data + 11), readLe32(data + 15));
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Cam %d: short UDP packet length=%zu",
                camera_index, length);
        }
        return;
    }
    const bool is_mor = magicIs(header, "MOR");
    const bool is_box = magicIs(header, "BOX");
    if (!is_mor && !is_box) return;
    if (header.packet_index > 100000U) {
        RCLCPP_WARN(get_logger(), "Cam %d: unreasonable packet index %u", camera_index, header.packet_index);
        return;
    }

    uint64_t timestamp_ns;
    if (!timestampNanoseconds(header, is_box, timestamp_ns)) {
        RCLCPP_WARN(get_logger(), "Cam %d: invalid MORAI timestamp fraction %u", camera_index, header.fraction);
        return;
    }

    const uint8_t* payload = data + kHeaderSize;
    const uint8_t* tail = data + length - kTailSize;
    const bool is_last = is_mor ? (tail[0] == 'E' && tail[1] == 'I') :
        (tail[0] == 'E' && tail[1] == 'O');
    const bool valid_tail = is_last ||
        (is_mor && tail[0] == 'A' && tail[1] == 'I');
    if (!valid_tail) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Cam %d: invalid %.3s packet tail", camera_index,
            header.magic.data());
        return;
    }

    std::unique_lock<std::mutex> lock(*buffer_mutexes_[camera_index]);
    auto& buffers = is_box ? box_buffers_[camera_index] : frame_buffers_[camera_index];

    if (buffers.find(timestamp_ns) == buffers.end()) {
        auto now = std::chrono::steady_clock::now();
        for (auto it = buffers.begin(); it != buffers.end(); ) {
            const double elapsed = std::chrono::duration<double>(now - it->second.last_update).count();
            if (elapsed > frame_timeout_sec_ || buffers.size() >= static_cast<size_t>(max_buffered_frames_)) {
                it = buffers.erase(it);
            } else {
                ++it;
            }
        }
    }

    FrameBuffer& frame = buffers[timestamp_ns];
    frame.timestamp_ns = timestamp_ns;
    frame.last_update = std::chrono::steady_clock::now();

    if (frame.packet_received.size() <= header.packet_index || !frame.packet_received[header.packet_index]) {
        frame.received_payloads[header.packet_index] =
            std::vector<uint8_t>(payload, payload + header.payload_size);
        if (frame.packet_received.size() <= header.packet_index) {
            frame.packet_received.resize(header.packet_index + 1, false);
        }
        frame.packet_received[header.packet_index] = true;
        frame.received_packets++;
        frame.is_jpeg = is_mor;
    }

    if (is_last) {
        frame.total_packets = header.packet_index + 1;
    }

    if (frame.total_packets > 0 && frame.received_packets >= frame.total_packets) {
        std::vector<uint8_t> assembled_data;
        size_t total_size = 0;
        for(const auto& pair : frame.received_payloads) { total_size += pair.second.size(); }
        assembled_data.reserve(total_size);

        bool complete = true;
        for (uint32_t i = 0; i < frame.total_packets; ++i) {
            if (frame.received_payloads.count(i)) {
                assembled_data.insert(
                    assembled_data.end(), frame.received_payloads[i].begin(), frame.received_payloads[i].end());
            } else {
                complete = false;
                break;
            }
        }

        if (complete) {
            if (is_box) {
                uint64_t image_timestamp_ns = 0;
                uint64_t best_difference = std::numeric_limits<uint64_t>::max();
                auto& image_timestamps = recent_image_timestamps_[camera_index];
                auto best = image_timestamps.end();
                for (auto it = image_timestamps.begin(); it != image_timestamps.end(); ++it) {
                    const uint64_t difference = *it > timestamp_ns ?
                        *it - timestamp_ns : timestamp_ns - *it;
                    if (difference < best_difference) {
                        best_difference = difference;
                        best = it;
                    }
                }
                const uint64_t tolerance_ns =
                    static_cast<uint64_t>(bbox_match_tolerance_ms_ * 1000000.0);
                if (best != image_timestamps.end() && best_difference <= tolerance_ns) {
                    image_timestamp_ns = *best;
                    image_timestamps.erase(best);
                }
                buffers.erase(timestamp_ns);
                lock.unlock();
                if (!image_timestamp_ns) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Cam %d: no MOR timestamp within %.1f ms of BOX timestamp %llu",
                        camera_index, bbox_match_tolerance_ms_,
                        static_cast<unsigned long long>(timestamp_ns));
                    return;
                }
                publishBoxes(camera_index, timestamp_ns, image_timestamp_ns, assembled_data);
                return;
            }

            auto& image_timestamps = recent_image_timestamps_[camera_index];
            image_timestamps.push_back(frame.timestamp_ns);
            while (image_timestamps.size() > 10) image_timestamps.pop_front();

            if (enable_sync_) {
                std::unique_lock<std::mutex> sync_lock(sync_mutex_);
                SyncFrame sync_frame;
                sync_frame.camera_index = camera_index;
                sync_frame.timestamp_ns = frame.timestamp_ns;
                sync_frame.is_jpeg = frame.is_jpeg;
                sync_frame.data = std::move(assembled_data);
                sync_frame.received_time = std::chrono::steady_clock::now();
                const uint64_t group_id = quantizeTimestamp(frame.timestamp_ns);
                synchronized_frames_[group_id].push_back(std::move(sync_frame));
                sync_lock.unlock();
                sync_cv_.notify_one();
            } else {
                // 동기화 비활성화 시에도 스레드풀 사용
                DecodeTask task;
                task.camera_index = camera_index;
                task.timestamp_ns = frame.timestamp_ns;
                task.is_jpeg = frame.is_jpeg;
                task.data = std::move(assembled_data);
                task.stamp = rclcpp::Time(static_cast<int64_t>(frame.timestamp_ns), RCL_ROS_TIME);
                queueDecodeTask(std::move(task));
            }
        }
        buffers.erase(timestamp_ns);
    }
}

void UdpCameraReceiver::publishBoxes(
    int camera_index, uint64_t box_timestamp_ns, uint64_t image_timestamp_ns,
    const std::vector<uint8_t>& data)
{
    std::vector<BoxObject> objects;
    std::string error;
    if (!parseBoxPayload(data.data(), data.size(), objects, error)) {
        RCLCPP_WARN(get_logger(), "Cam %d: dropping BOX frame: %s", camera_index, error.c_str());
        return;
    }

    // The UDP page does not name the four fields. MORAI's documented saved 2D format is
    // x1,y1,x2,y2; keep this assumption visible and verify it with debug_overlay on real data.
    for (const auto& object : objects) {
        if (object.bbox_2d_raw[2] - object.bbox_2d_raw[0] < 0.0F ||
            object.bbox_2d_raw[3] - object.bbox_2d_raw[1] < 0.0F) {
            RCLCPP_WARN(get_logger(), "Cam %d: dropping BOX frame with negative bbox size", camera_index);
            return;
        }
    }

    vision_msgs::msg::Detection2DArray message;
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(image_timestamp_ns), RCL_ROS_TIME);
    message.header.frame_id = cameras_[camera_index].name;
    message.detections.reserve(objects.size());
    for (const auto& object : objects) {
        vision_msgs::msg::Detection2D detection;
        detection.header = message.header;
        detection.bbox.center.position.x =
            (object.bbox_2d_raw[0] + object.bbox_2d_raw[2]) * 0.5;
        detection.bbox.center.position.y =
            (object.bbox_2d_raw[1] + object.bbox_2d_raw[3]) * 0.5;
        detection.bbox.size_x = object.bbox_2d_raw[2] - object.bbox_2d_raw[0];
        detection.bbox.size_y = object.bbox_2d_raw[3] - object.bbox_2d_raw[1];
        detection.results.emplace_back();
        detection.results[0].hypothesis.class_id = boxClassLabel(object.class_tag);
        detection.results[0].hypothesis.score = 1.0;
        message.detections.push_back(std::move(detection));
    }
    detection_publishers_[camera_index]->publish(message);

    if (debug_mode_) {
        if (objects.empty()) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000, "Cam %d BOX timestamp=%llu objects=0",
                camera_index, static_cast<unsigned long long>(box_timestamp_ns));
        } else {
            std::ostringstream details;
            details << std::fixed << std::setprecision(3);
            for (size_t i = 0; i < objects.size(); ++i) {
                const auto& box = objects[i].bbox_2d_raw;
                const size_t tag = i * kBoxObjectSize + 112;
                details << " #" << i << " tag=" << std::hex << std::setfill('0')
                        << std::setw(2) << static_cast<unsigned>(data[tag])
                        << std::setw(2) << static_cast<unsigned>(data[tag + 1])
                        << std::setw(2) << static_cast<unsigned>(data[tag + 2])
                        << std::dec << " bbox=[" << box[0] << ' ' << box[1] << ' '
                        << box[2] << ' ' << box[3] << ']';
            }
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Cam %d BOX timestamp=%llu objects=%zu%s",
                camera_index, static_cast<unsigned long long>(box_timestamp_ns), objects.size(),
                details.str().c_str());
        }
    }

    if (publish_bbox_overlay_) {
        {
            std::lock_guard<std::mutex> lock(overlay_mutex_);
            auto& boxes = overlay_boxes_[camera_index];
            boxes[image_timestamp_ns] = objects;
            while (boxes.size() > static_cast<size_t>(max_buffered_frames_)) boxes.erase(boxes.begin());
        }
        publishOverlayIfReady(camera_index, image_timestamp_ns);
    }
}

void UdpCameraReceiver::publishOverlayIfReady(int camera_index, uint64_t timestamp_ns)
{
    cv::Mat image;
    std::vector<BoxObject> objects;
    {
        std::lock_guard<std::mutex> lock(overlay_mutex_);
        auto image_it = overlay_images_[camera_index].find(timestamp_ns);
        auto boxes_it = overlay_boxes_[camera_index].find(timestamp_ns);
        if (image_it == overlay_images_[camera_index].end() ||
            boxes_it == overlay_boxes_[camera_index].end()) return;
        image = std::move(image_it->second);
        objects = std::move(boxes_it->second);
        overlay_images_[camera_index].erase(image_it);
        overlay_boxes_[camera_index].erase(boxes_it);
    }

    for (const auto& object : objects) {
        cv::rectangle(
            image, cv::Point2f(object.bbox_2d_raw[0], object.bbox_2d_raw[1]),
            cv::Point2f(object.bbox_2d_raw[2], object.bbox_2d_raw[3]), cv::Scalar(255, 0, 0), 2);
        cv::putText(
            image, boxClassLabel(object.class_tag),
            cv::Point(static_cast<int>(object.bbox_2d_raw[0]),
                std::max(12, static_cast<int>(object.bbox_2d_raw[1]))),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp_ns), RCL_ROS_TIME);
    header.frame_id = cameras_[camera_index].name;
    overlay_publishers_[camera_index]->publish(*cv_bridge::CvImage(header, "rgb8", image).toImageMsg());
}

void UdpCameraReceiver::synchronizerThread()
{
    while (running_ && rclcpp::ok()) {
        std::vector<DecodeTask> tasks_to_queue;

        {
            std::unique_lock<std::mutex> lock(sync_mutex_);
            sync_cv_.wait_for(lock, std::chrono::milliseconds(static_cast<int>(sync_timeout_sec_ * 1000)));

            if (!running_) break;

            auto now = std::chrono::steady_clock::now();
            for (auto it = synchronized_frames_.begin(); it != synchronized_frames_.end(); ) {
                // 중복 제거: 각 카메라에서 가장 최신 프레임만 유지
                std::map<int, size_t> unique_cameras;  // camera_index -> index in it->second
                for (size_t idx = 0; idx < it->second.size(); ++idx) {
                    const auto& frame = it->second[idx];
                    auto existing = unique_cameras.find(frame.camera_index);
                    if (existing == unique_cameras.end() ||
                        frame.received_time > it->second[existing->second].received_time) {
                        unique_cameras[frame.camera_index] = idx;
                    }
                }

                bool all_cameras_received = (unique_cameras.size() == cameras_.size());
                bool timeout_exceeded = false;

                if (!it->second.empty()) {
                    auto first_frame_time = it->second.front().received_time;
                    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - first_frame_time).count();
                    if (elapsed > sync_timeout_sec_) {
                        timeout_exceeded = true;
                    }
                }

                if (all_cameras_received) {
                    if (debug_mode_) {
                        RCLCPP_INFO(
                            this->get_logger(), "Sync: Publishing complete frame group %llu",
                            static_cast<unsigned long long>(it->first));
                    }
                    // 락 해제 전에 데이터를 move하여 복사
                    for (const auto& pair : unique_cameras) {
                        auto& frame = it->second[pair.second];
                        DecodeTask task;
                        task.camera_index = frame.camera_index;
                        task.timestamp_ns = frame.timestamp_ns;
                        task.is_jpeg = frame.is_jpeg;
                        task.data = std::move(frame.data);
                        task.stamp = rclcpp::Time(
                            static_cast<int64_t>(frame.timestamp_ns), RCL_ROS_TIME);
                        tasks_to_queue.push_back(std::move(task));
                    }
                    it = synchronized_frames_.erase(it);
                } else if (timeout_exceeded) {
                    // 어떤 카메라가 누락됐는지 로그
                    std::string missing_cams;
                    for (size_t i = 0; i < cameras_.size(); ++i) {
                        if (unique_cameras.find(i) == unique_cameras.end()) {
                            if (!missing_cams.empty()) missing_cams += ",";
                            missing_cams += std::to_string(i) + "(" + cameras_[i].name + ")";
                        }
                    }
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Sync: Timeout on group %llu. Received %zu/%zu. Missing: %s",
                        static_cast<unsigned long long>(it->first), unique_cameras.size(),
                        cameras_.size(), missing_cams.c_str());
                    it = synchronized_frames_.erase(it);
                } else {
                    ++it;
                }
            }
        }  // sync_mutex_ 락 해제

        // 락 없이 디코딩 작업 큐에 추가
        for (auto& task : tasks_to_queue) {
            queueDecodeTask(std::move(task));
        }
    }
}


void UdpCameraReceiver::queueDecodeTask(DecodeTask&& task)
{
    {
        std::lock_guard<std::mutex> lock(decode_mutex_);
        decode_queue_.push(std::move(task));
    }
    decode_cv_.notify_one();
}

void UdpCameraReceiver::decodeWorkerThread(int worker_id)
{
    RCLCPP_INFO(this->get_logger(), "Decode worker %d started (GPU: %s)",
                worker_id, nvjpeg_initialized_ ? "enabled" : "disabled");

    while (running_ && rclcpp::ok()) {
        DecodeTask task;
        {
            std::unique_lock<std::mutex> lock(decode_mutex_);
            decode_cv_.wait(lock, [this] {
                return !decode_queue_.empty() || !running_;
            });

            if (!running_ && decode_queue_.empty()) break;
            if (decode_queue_.empty()) continue;

            task = std::move(decode_queue_.front());
            decode_queue_.pop();
        }

        const auto& config = cameras_[task.camera_index];

        if (config.compressed) {
            if (!task.is_jpeg) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Cam %d: compressed output requires JPEG input",
                    task.camera_index);
                continue;
            }

            cv::Mat overlay_image;
            if (publish_bbox_overlay_) {
                overlay_image = cv::imdecode(task.data, cv::IMREAD_COLOR);
                if (!overlay_image.empty()) {
                    cv::cvtColor(overlay_image, overlay_image, cv::COLOR_BGR2RGB);
                }
            }

            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header.stamp = task.stamp;
            msg->header.frame_id = config.name;
            msg->format = "jpeg";
            msg->data = std::move(task.data);
            compressed_publishers_[task.camera_index]->publish(std::move(msg));

            if (publish_camera_info_) {
                auto info_msg = camera_info_msgs_[task.camera_index];
                info_msg.header.stamp = task.stamp;
                info_msg.header.frame_id = config.name;
                camera_info_publishers_[task.camera_index]->publish(info_msg);
            }
            if (!overlay_image.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(overlay_mutex_);
                        auto& images = overlay_images_[task.camera_index];
                        images[task.timestamp_ns] = std::move(overlay_image);
                        while (images.size() > static_cast<size_t>(max_buffered_frames_)) {
                            images.erase(images.begin());
                        }
                    }
                    publishOverlayIfReady(task.camera_index, task.timestamp_ns);
            }
            continue;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = task.stamp;
        msg->header.frame_id = config.name;

        if (task.is_jpeg) {
            // GPU 디코딩 시도
            std::vector<uint8_t> rgb_data;
            int width, height;
            bool gpu_success = decodeJpegGpu(worker_id, task.data, rgb_data, width, height);

            if (gpu_success) {
                msg->height = height;
                msg->width = width;
                msg->encoding = "rgb8";
                msg->is_bigendian = false;
                msg->step = width * 3;
                msg->data = std::move(rgb_data);
            } else {
                // CPU 폴백
                cv::Mat image = cv::imdecode(task.data, cv::IMREAD_COLOR);
                if (image.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Cam %d: Failed to decode JPEG (size=%zu)",
                                task.camera_index, task.data.size());
                    continue;
                }
                cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

                msg->height = image.rows;
                msg->width = image.cols;
                msg->encoding = "rgb8";
                msg->is_bigendian = false;
                msg->step = image.cols * image.elemSize();

                size_t data_size = msg->step * image.rows;
                msg->data.resize(data_size);
                if (image.isContinuous()) {
                    std::memcpy(msg->data.data(), image.data, data_size);
                } else {
                    for (int row = 0; row < image.rows; ++row) {
                        std::memcpy(msg->data.data() + row * msg->step, image.ptr(row), msg->step);
                    }
                }
            }
        } else {
            msg->height = config.height;
            msg->width = config.width;
            msg->is_bigendian = false;

            if (config.channels == 3) {
                msg->encoding = "rgb8";
                msg->step = config.width * 3;
            } else {
                msg->encoding = "mono8";
                msg->step = config.width;
            }

            msg->data = std::move(task.data);
            if (config.channels == 3) {
                for (size_t i = 0; i + 2 < msg->data.size(); i += 3) {
                    std::swap(msg->data[i], msg->data[i + 2]);
                }
            }
        }

        if (publish_bbox_overlay_) {
            const int type = msg->encoding == "rgb8" ? CV_8UC3 : CV_8UC1;
            cv::Mat image(static_cast<int>(msg->height), static_cast<int>(msg->width), type,
                msg->data.data(), msg->step);
            cv::Mat overlay_image;
            if (type == CV_8UC3) {
                overlay_image = image.clone();
            } else {
                cv::cvtColor(image, overlay_image, cv::COLOR_GRAY2RGB);
            }
            {
                std::lock_guard<std::mutex> lock(overlay_mutex_);
                auto& images = overlay_images_[task.camera_index];
                images[task.timestamp_ns] = std::move(overlay_image);
                while (images.size() > static_cast<size_t>(max_buffered_frames_)) {
                    images.erase(images.begin());
                }
            }
            publishOverlayIfReady(task.camera_index, task.timestamp_ns);
        }

        publishers_[task.camera_index]->publish(std::move(msg));

        if (publish_camera_info_) {
            auto info_msg = camera_info_msgs_[task.camera_index];
            info_msg.header.stamp = task.stamp;
            info_msg.header.frame_id = config.name;
            camera_info_publishers_[task.camera_index]->publish(info_msg);
        }

        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Cam %d: Published timestamp %llu (worker %d)",
                       task.camera_index,
                       static_cast<unsigned long long>(task.timestamp_ns), worker_id);
        }
    }
}

}  // namespace udp_camera_receiver
