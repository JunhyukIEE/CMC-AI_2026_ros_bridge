#include "udp_camera_receiver/udp_camera_receiver.hpp"

#include <gtest/gtest.h>

namespace udp_camera_receiver
{
// Exercise the real assembly, sync, decode and publishing paths without UDP input.
class UdpCameraReceiverTimestampTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void start(bool sync, bool morai = false)
    {
        auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
        options.parameter_overrides({rclcpp::Parameter("use_morai_timestamp", morai)});
        // No camera_N parameters: constructor starts workers but binds no UDP ports.
        receiver_ = std::make_shared<UdpCameraReceiver>(options);
        observer_ = std::make_shared<rclcpp::Node>("timestamp_test_observer", options);
        receiver_->enable_sync_ = sync;
        receiver_->sync_timeout_sec_ = 1.0;
        receiver_->publish_bbox_overlay_ = true;
        const auto qos = rclcpp::SensorDataQoS().keep_last(1);
        for (int i = 0; i < 2; ++i) {
            const std::string topic = "/timestamp_test/camera" + std::to_string(i);
            CameraConfig config{};
            config.name = "optical" + std::to_string(i);
            config.compressed = (i == 1);
            receiver_->cameras_.push_back(config);
            receiver_->publishers_.push_back(config.compressed ? nullptr :
                receiver_->create_publisher<sensor_msgs::msg::Image>(topic + "/image", qos));
            receiver_->compressed_publishers_.push_back(config.compressed ?
                receiver_->create_publisher<sensor_msgs::msg::CompressedImage>(topic + "/image", qos) : nullptr);
            receiver_->camera_info_publishers_.push_back(
                receiver_->create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/info", qos));
            receiver_->camera_info_msgs_.emplace_back();
            receiver_->detection_publishers_.push_back(
                receiver_->create_publisher<vision_msgs::msg::Detection2DArray>(topic + "/boxes", qos));
            receiver_->overlay_publishers_.push_back(
                receiver_->create_publisher<sensor_msgs::msg::Image>(topic + "/overlay", qos));
            receiver_->frame_buffers_.emplace_back();
            receiver_->box_buffers_.emplace_back();
            receiver_->recent_image_timestamps_.emplace_back();
            receiver_->overlay_images_.emplace_back();
            receiver_->overlay_boxes_.emplace_back();
            receiver_->buffer_mutexes_.push_back(std::make_unique<std::mutex>());
            if (config.compressed) observe<sensor_msgs::msg::CompressedImage>(i, topic, "image");
            else observe<sensor_msgs::msg::Image>(i, topic, "image");
            observe<sensor_msgs::msg::CameraInfo>(i, topic, "info");
            observe<vision_msgs::msg::Detection2DArray>(i, topic, "boxes");
            observe<sensor_msgs::msg::Image>(i, topic, "overlay");
        }
        executor_.add_node(receiver_);
        executor_.add_node(observer_);
    }

    template<class Message>
    void observe(int camera, const std::string& topic, const std::string& label)
    {
        subscriptions_.push_back(observer_->create_subscription<Message>(
            topic + "/" + label, rclcpp::SensorDataQoS(),
            [this, camera, label](typename Message::ConstSharedPtr message) {
                received_[camera][label] = rclcpp::Time(message->header.stamp).nanoseconds();
                EXPECT_EQ(message->header.frame_id, "optical" + std::to_string(camera));
            }));
    }

    void packet(int camera, bool box, uint32_t fraction, uint32_t index,
                const std::vector<uint8_t>& payload, bool last, int64_t receive_ns)
    {
        std::vector<uint8_t> bytes(19);
        const std::string magic = box ? "BOX" : "MOR";
        std::copy(magic.begin(), magic.end(), bytes.begin());
        auto le32 = [&bytes](size_t offset, uint32_t value) {
            for (int i = 0; i < 4; ++i) bytes[offset + i] = (value >> (8 * i)) & 0xff;
        };
        le32(3, 1000); le32(7, fraction); le32(11, index); le32(15, payload.size());
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        bytes.push_back(last ? 'E' : 'A');
        bytes.push_back(box ? 'O' : 'I');
        receiver_->processPacket(camera, bytes.data(), bytes.size(), rclcpp::Time(receive_ns, RCL_ROS_TIME));
    }

    void frame(int camera, uint32_t fraction, int64_t first_ns)
    {
        std::vector<uint8_t> jpeg;
        ASSERT_TRUE(cv::imencode(".jpg", cv::Mat(16, 16, CV_8UC3, cv::Scalar(40, 80, 120)), jpeg));
        const auto middle = jpeg.begin() + jpeg.size() / 2;
        const std::vector<uint8_t> head(jpeg.begin(), middle), tail(middle, jpeg.end());
        // Last fragment arrives FIRST, then a duplicate, then index 0.
        packet(camera, false, fraction, 1, tail, true, first_ns);
        packet(camera, false, fraction, 1, tail, true, first_ns + 20000000);
        {
            std::lock_guard<std::mutex> lock(*receiver_->buffer_mutexes_[camera]);
            const auto& f = receiver_->frame_buffers_[camera].at(1000000000000ULL + fraction);
            EXPECT_EQ(f.received_packets, 1U);
            EXPECT_EQ(f.stamp.nanoseconds(), receiver_->use_morai_timestamp_ ?
                1000000000000LL + fraction : first_ns);
        }
        packet(camera, false, fraction, 0, head, false, first_ns + 40000000);
    }

    void boxes(int camera, uint32_t fraction)
    {
        // One finite, zero-size GT bbox. BOX time differs from MOR by 5 ms;
        // its own local receive time must never become the detection stamp.
        std::vector<uint8_t> payload(115, 0);
        payload[112] = 0xff; payload[113] = 2; payload[114] = 2;
        packet(camera, true, fraction + 5000000, 0, payload, true, 99000000000LL);
    }

    void startSync()
    {
        receiver_->synchronizer_thread_ = std::make_shared<std::thread>(
            &UdpCameraReceiver::synchronizerThread, receiver_.get());
        receiver_->sync_cv_.notify_one();
    }

    void waitFor(int camera, size_t count)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (received_[camera].size() < count && std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        ASSERT_EQ(received_[camera].size(), count);
    }

    void check(int camera, int64_t expected)
    {
        waitFor(camera, 4);
        for (const auto& value : received_[camera]) EXPECT_EQ(value.second, expected) << value.first;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<UdpCameraReceiver> receiver_;
    rclcpp::Node::SharedPtr observer_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::array<std::map<std::string, int64_t>, 2> received_;
};

TEST_F(UdpCameraReceiverTimestampTest, FirstReceptionSurvivesAssemblyAndBothOutputPaths)
{
    start(false);
    frame(0, 123000000, 10000000000LL);
    waitFor(0, 2);  // Decode first, then BOX: exercise the other overlay arrival order.
    boxes(0, 123000000);
    frame(1, 130000000, 20000000000LL);
    waitFor(1, 2);
    boxes(1, 130000000);
    check(0, 10000000000LL);
    check(1, 20000000000LL);
}

TEST_F(UdpCameraReceiverTimestampTest, SyncUsesMoraiKeysButKeepsEachCamerasReceptionStamp)
{
    start(true);
    frame(0, 123000000, 10000000000LL);
    boxes(0, 123000000);  // BOX before decoding.
    frame(1, 130000000, 20000000000LL);
    boxes(1, 130000000);
    startSync();
    check(0, 10000000000LL);
    check(1, 20000000000LL);
}

TEST_F(UdpCameraReceiverTimestampTest, OptionalMoraiClockPreservesOriginalStamps)
{
    start(false, true);
    frame(0, 123000000, 10000000000LL);
    boxes(0, 123000000);
    frame(1, 130000000, 20000000000LL);
    boxes(1, 130000000);
    check(0, 1000123000000LL);
    check(1, 1000130000000LL);
}
}  // namespace udp_camera_receiver
