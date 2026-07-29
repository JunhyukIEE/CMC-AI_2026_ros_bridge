#include "udp_camera_receiver/udp_camera_receiver.hpp"
#include <cstring>
#include <cmath>

namespace udp_camera_receiver
{

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
    this->declare_parameter<int>("max_buffered_frames", 3);
    this->declare_parameter<double>("frame_timeout_sec", 1.0);
    this->declare_parameter<bool>("publish_camera_info", true);
    this->declare_parameter<double>("hfov_deg", 90.0);
    this->declare_parameter<bool>("enable_sync", false);
    this->declare_parameter<double>("sync_timeout_sec", 0.1);
    this->declare_parameter<double>("sync_window_ms", 30.0);  // 기본 30ms 윈도우

    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("max_buffered_frames", max_buffered_frames_);
    this->get_parameter("frame_timeout_sec", frame_timeout_sec_);
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

        cameras_.push_back(config);

        RCLCPP_INFO(this->get_logger(), "Camera %d: %s - %s:%d -> %s (%dx%d)",
                    i, config.name.c_str(), config.ip.c_str(), config.port,
                    config.topic_name.c_str(), config.width, config.height);
    }
}

void UdpCameraReceiver::initializeCameras()
{
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
                    config.topic_name, 10));
        } else {
            publishers_.push_back(
                this->create_publisher<sensor_msgs::msg::Image>(
                    config.topic_name, 10));
            compressed_publishers_.push_back(nullptr);
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

            auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);
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
        frame_buffers_.push_back(std::map<uint32_t, FrameBuffer>());
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
                                       std::vector<uint8_t>& bgr_data, int& width, int& height)
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

    // BGR interleaved 출력 (3채널)
    size_t pitch = width * 3;
    unsigned char* d_output = nullptr;
    cudaError_t cuda_status = cudaMalloc(&d_output, pitch * height);
    if (cuda_status != cudaSuccess) {
        return false;
    }

    output_image.channel[0] = d_output;
    output_image.pitch[0] = pitch;

    // 디코딩 (BGR interleaved)
    status = nvjpegDecode(
        nvjpeg_handle_, nvjpeg_states_[worker_id],
        jpeg_data.data(), jpeg_data.size(),
        NVJPEG_OUTPUT_BGRI, &output_image,
        cuda_streams_[worker_id]);

    if (status != NVJPEG_STATUS_SUCCESS) {
        cudaFree(d_output);
        return false;
    }

    // 스트림 동기화
    cudaStreamSynchronize(cuda_streams_[worker_id]);

    // GPU -> CPU 복사
    bgr_data.resize(pitch * height);
    cuda_status = cudaMemcpy(bgr_data.data(), d_output, pitch * height, cudaMemcpyDeviceToHost);
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
    if (length < sizeof(PacketHeader)) return;

    const PacketHeader* header = reinterpret_cast<const PacketHeader*>(data);
    bool is_mor = (strncmp(header->magic, "MOR", 3) == 0);
    if (!is_mor) return;

    const uint8_t* payload = data + sizeof(PacketHeader);
    size_t payload_size = header->payload_size;

    std::unique_lock<std::mutex> lock(*buffer_mutexes_[camera_index]);
    auto& buffers = frame_buffers_[camera_index];

    if (buffers.find(header->frame_id) == buffers.end()) {
        auto now = std::chrono::steady_clock::now();
        for (auto it = buffers.begin(); it != buffers.end(); ) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update).count();
            if (elapsed > frame_timeout_sec_ || buffers.size() >= static_cast<size_t>(max_buffered_frames_)) {
                it = buffers.erase(it);
            } else {
                ++it;
            }
        }
    }

    FrameBuffer& frame = buffers[header->frame_id];
    frame.frame_number = header->frame_id;
    frame.last_update = std::chrono::steady_clock::now();

    if (frame.packet_received.size() <= header->packet_number || !frame.packet_received[header->packet_number]) {
        frame.received_payloads[header->packet_number] = std::vector<uint8_t>(payload, payload + payload_size);
        if (frame.packet_received.size() <= header->packet_number) {
            frame.packet_received.resize(header->packet_number + 1, false);
        }
        frame.packet_received[header->packet_number] = true;
        frame.received_packets++;
        frame.is_jpeg = is_mor;
    }

    bool is_last_packet = false;
    if (payload_size < 2 || (payload[payload_size - 2] == 0xFF && payload[payload_size - 1] == 0xD9)) {
        for (int i = payload_size - 2; i >= 0; --i) {
            if (payload[i] == 0xFF && payload[i+1] == 0xD9) {
                is_last_packet = true;
                break;
            }
        }
    }
    
    if (is_last_packet) {
        frame.total_packets = header->packet_number + 1;
    }

    if (frame.total_packets > 0 && frame.received_packets >= frame.total_packets) {
        std::vector<uint8_t> image_data;
        size_t total_size = 0;
        for(const auto& pair : frame.received_payloads) { total_size += pair.second.size(); }
        image_data.reserve(total_size);

        bool complete = true;
        for (uint32_t i = 0; i < frame.total_packets; ++i) {
            if (frame.received_payloads.count(i)) {
                image_data.insert(image_data.end(), frame.received_payloads[i].begin(), frame.received_payloads[i].end());
            } else {
                complete = false;
                break;
            }
        }
        
        if (complete) {
            if (enable_sync_) {
                std::unique_lock<std::mutex> sync_lock(sync_mutex_);
                SyncFrame sync_frame;
                sync_frame.camera_index = camera_index;
                sync_frame.frame_id = frame.frame_number;
                sync_frame.is_jpeg = frame.is_jpeg;
                sync_frame.data = std::move(image_data);
                sync_frame.received_time = std::chrono::steady_clock::now();
                // 양자화된 frame_id를 그룹 키로 사용
                uint32_t group_id = quantizeFrameId(frame.frame_number);
                synchronized_frames_[group_id].push_back(std::move(sync_frame));
                sync_lock.unlock();
                sync_cv_.notify_one();
            } else {
                // 동기화 비활성화 시에도 스레드풀 사용
                DecodeTask task;
                task.camera_index = camera_index;
                task.frame_id = frame.frame_number;
                task.is_jpeg = frame.is_jpeg;
                task.data = std::move(image_data);
                task.stamp = this->now();
                queueDecodeTask(std::move(task));
            }
        }
        buffers.erase(header->frame_id);
    }
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
            auto stamp = this->now();

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
                        RCLCPP_INFO(this->get_logger(), "Sync: Publishing complete frame group %u", it->first);
                    }
                    // 락 해제 전에 데이터를 move하여 복사
                    for (const auto& pair : unique_cameras) {
                        auto& frame = it->second[pair.second];
                        DecodeTask task;
                        task.camera_index = frame.camera_index;
                        task.frame_id = frame.frame_id;
                        task.is_jpeg = frame.is_jpeg;
                        task.data = std::move(frame.data);
                        task.stamp = stamp;
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
                    RCLCPP_WARN(this->get_logger(), "Sync: Timeout on group %u. Received %zu/%zu. Missing: %s",
                                it->first, unique_cameras.size(), cameras_.size(), missing_cams.c_str());
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
            continue;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = task.stamp;
        msg->header.frame_id = config.name;

        if (task.is_jpeg) {
            // GPU 디코딩 시도
            std::vector<uint8_t> bgr_data;
            int width, height;
            bool gpu_success = decodeJpegGpu(worker_id, task.data, bgr_data, width, height);

            if (gpu_success) {
                msg->height = height;
                msg->width = width;
                msg->encoding = "bgr8";
                msg->is_bigendian = false;
                msg->step = width * 3;
                msg->data = std::move(bgr_data);
            } else {
                // CPU 폴백
                cv::Mat image = cv::imdecode(task.data, cv::IMREAD_COLOR);
                if (image.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Cam %d: Failed to decode JPEG (size=%zu)",
                                task.camera_index, task.data.size());
                    continue;
                }

                msg->height = image.rows;
                msg->width = image.cols;
                msg->encoding = "bgr8";
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
                msg->encoding = "bgr8";
                msg->step = config.width * 3;
            } else {
                msg->encoding = "mono8";
                msg->step = config.width;
            }

            msg->data = std::move(task.data);
        }

        publishers_[task.camera_index]->publish(std::move(msg));

        if (publish_camera_info_) {
            auto info_msg = camera_info_msgs_[task.camera_index];
            info_msg.header.stamp = task.stamp;
            info_msg.header.frame_id = config.name;
            camera_info_publishers_[task.camera_index]->publish(info_msg);
        }

        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(), "Cam %d: Published frame %u (worker %d)",
                       task.camera_index, task.frame_id, worker_id);
        }
    }
}

void UdpCameraReceiver::publishImage(int camera_index, uint32_t frame_id, bool is_jpeg, const std::vector<uint8_t>& data, const rclcpp::Time& stamp)
{
    const auto& config = cameras_[camera_index];

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = config.name;

    if (is_jpeg) {
        cv::Mat image = cv::imdecode(data, cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Cam %d: Failed to decode JPEG (size=%zu)", camera_index, data.size());
            return;
        }

        msg->height = image.rows;
        msg->width = image.cols;
        msg->encoding = "bgr8";
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
    } else {
        msg->height = config.height;
        msg->width = config.width;
        msg->is_bigendian = false;

        if (config.channels == 3) {
            msg->encoding = "bgr8";
            msg->step = config.width * 3;
        } else {
            msg->encoding = "mono8";
            msg->step = config.width;
        }

        msg->data = data;  // copy (필요시 외부에서 move 가능)
    }

    publishers_[camera_index]->publish(std::move(msg));

    if (publish_camera_info_) {
        auto info_msg = camera_info_msgs_[camera_index];
        info_msg.header.stamp = stamp;
        info_msg.header.frame_id = config.name;
        camera_info_publishers_[camera_index]->publish(info_msg);
    }

    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), "Cam %d: Published frame %u", camera_index, frame_id);
    }
}

}  // namespace udp_camera_receiver
