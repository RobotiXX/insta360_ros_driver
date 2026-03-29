#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class H264StreamDelegate : public ins_camera::StreamDelegate {
public:
    H264StreamDelegate(rclcpp::Node* node, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub);
    ~H264StreamDelegate();

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr h264_pub_;
    std::mutex pub_mutex_;
};

H264StreamDelegate::H264StreamDelegate(
    rclcpp::Node* node,
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub)
    : node_(node), h264_pub_(pub)
{
}

H264StreamDelegate::~H264StreamDelegate()
{
}

void H264StreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

void H264StreamDelegate::OnVideoData(
    const uint8_t* data,
    size_t size,
    int64_t timestamp,
    uint8_t streamType,
    int stream_index)
{
    (void)timestamp;
    (void)streamType;

    if (stream_index != 0 || size == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(pub_mutex_);

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "camera_frame";
    msg->format = "h264";
    msg->data.assign(data, data + size);

    h264_pub_->publish(std::move(msg));
}

void H264StreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    (void)data;
}

void H264StreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

class H264PublisherNode : public rclcpp::Node {
public:
    H264PublisherNode()
        : Node("h264_publisher")
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);

        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();

        h264_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/h264", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "H264PublisherNode initialized.");
    }

    bool startCamera()
    {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(get_logger(), "No available camera devices found.");
            return false;
        }

        cam_ = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam_->Open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera.");
            discovery.FreeDeviceDescriptors(list);
            return false;
        }
        discovery.FreeDeviceDescriptors(list);

        std::shared_ptr<ins_camera::StreamDelegate> delegate =
            std::make_shared<H264StreamDelegate>(this, h264_pub_);
        stream_delegate_ = delegate;
        cam_->SetStreamDelegate(delegate);

        uint64_t utc_time = static_cast<uint64_t>(time(NULL));
        cam_->SyncLocalTimeToCamera(utc_time);

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_2880_1440P30;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Started H.264 publisher node.");
        return true;
    }

    ~H264PublisherNode() override
    {
        if (cam_) {
            cam_->Close();
        }
    }

private:
    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr h264_pub_;

    int skip_frame_;
    bool i_frame_only_;

    friend class H264StreamDelegate;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<H264PublisherNode>();
        if (!node->startCamera()) {
            rclcpp::shutdown();
            return -1;
        }

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("h264_publisher");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
