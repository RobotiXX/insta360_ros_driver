#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "NvBuffer.h"
#include "NvVideoDecoder.h"

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define MMAPI_CHUNK_SIZE (4 * 1024 * 1024)

class MmapiDecodedNode;

class MmapiDecodedStreamDelegate : public ins_camera::StreamDelegate {
public:
    MmapiDecodedStreamDelegate(MmapiDecodedNode* owner, int skip_frame, bool i_frame_only);
    ~MmapiDecodedStreamDelegate();

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    bool isIdrFrame(const uint8_t* data, size_t size) const;
    void initDecoder();
    bool queueBitstream(const uint8_t* data, size_t size);
    void captureLoop();
    bool setupCapturePlane();
    bool processCapturedFrame(NvBuffer* buffer);
    void cleanupDecoder();

    MmapiDecodedNode* owner_;

    cv::Mat bgr_frame_;

    NvVideoDecoder* decoder_ = nullptr;
    std::thread capture_thread_;
    std::atomic<bool> capture_running_{false};
    std::atomic<bool> capture_setup_done_{false};
    bool resolution_event_seen_ = false;
    uint32_t capture_width_ = 0;
    uint32_t capture_height_ = 0;
    uint32_t output_plane_num_buffers_ = 0;
    uint32_t output_plane_next_index_ = 0;
    uint32_t output_plane_queued_ = 0;

    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool decoder_ready_ = false;
    bool wait_for_first_idr_ = true;
    bool resolution_event_failed_ = false;

    std::mutex decoder_mutex_;
};

class MmapiDecodedNode : public rclcpp::Node {
public:
    MmapiDecodedNode()
        : Node("insta360_decoded_mmapi_node")
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);

        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS());
    }

    ~MmapiDecodedNode() override
    {
        if (cam_) {
            cam_->Close();
        }
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

        stream_delegate_ = std::make_shared<MmapiDecodedStreamDelegate>(
            this,
            skip_frame_,
            i_frame_only_);
        cam_->SetStreamDelegate(stream_delegate_);

        const uint64_t utc_time = static_cast<uint64_t>(time(nullptr));
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

        RCLCPP_INFO(get_logger(), "Started integrated Jetson MMAPI decode node.");
        return true;
    }

    void publishDecodedFrame(const cv::Mat& decoded_bgr)
    {
        if (decoded_bgr.empty()) {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera_frame";

        cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, decoded_bgr);
        auto out_msg = cv_image.toImageMsg();
        image_pub_->publish(*out_msg);
    }

    void publishImu(const ins_camera::GyroData& gyro)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = now();
        msg->header.frame_id = "imu_frame";

        msg->angular_velocity.x = gyro.gx;
        msg->angular_velocity.y = gyro.gy;
        msg->angular_velocity.z = gyro.gz;

        msg->linear_acceleration.x = gyro.ax * 9.80665;
        msg->linear_acceleration.y = gyro.ay * 9.80665;
        msg->linear_acceleration.z = gyro.az * 9.80665;

        msg->orientation.x = 0.0;
        msg->orientation.y = 0.0;
        msg->orientation.z = 0.0;
        msg->orientation.w = 1.0;
        msg->orientation_covariance[0] = -1.0;

        for (int i = 0; i < 9; ++i) {
            msg->angular_velocity_covariance[i] = 0.0;
            msg->linear_acceleration_covariance[i] = 0.0;
        }

        imu_pub_->publish(std::move(msg));
    }

private:
    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    int skip_frame_ = 0;
    bool i_frame_only_ = false;

    friend class MmapiDecodedStreamDelegate;
};

MmapiDecodedStreamDelegate::MmapiDecodedStreamDelegate(
    MmapiDecodedNode* owner,
    int skip_frame,
    bool i_frame_only)
    : owner_(owner),
      skip_frame_(skip_frame),
      i_frame_only_(i_frame_only)
{
    initDecoder();
}

MmapiDecodedStreamDelegate::~MmapiDecodedStreamDelegate()
{
    cleanupDecoder();
}

void MmapiDecodedStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

bool MmapiDecodedStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
{
    for (size_t i = 0; i + 4 < size; ++i) {
        const bool start3 = (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01);
        const bool start4 =
            (i + 5 < size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00 && data[i + 3] == 0x01);
        if (!start3 && !start4) {
            continue;
        }

        const size_t nal_idx = start3 ? i + 3 : i + 4;
        if (nal_idx >= size) {
            break;
        }

        const uint8_t nal_type = data[nal_idx] & 0x1f;
        if (nal_type == 5) {
            return true;
        }
    }

    return false;
}

void MmapiDecodedStreamDelegate::OnVideoData(
    const uint8_t* data,
    size_t size,
    int64_t timestamp,
    uint8_t streamType,
    int stream_index)
{
    (void)timestamp;
    (void)streamType;

    if (stream_index != 0 || size == 0 || !decoder_ready_) {
        return;
    }

    if (wait_for_first_idr_) {
        if (!isIdrFrame(data, size)) {
            return;
        }
        wait_for_first_idr_ = false;
        RCLCPP_INFO(owner_->get_logger(), "Received first IDR frame, starting MMAPI decode feed.");
    }

    if (i_frame_only_ && !isIdrFrame(data, size)) {
        return;
    }

    if (skip_frame_ > 0 && !i_frame_only_) {
        const bool should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
        if (!should_publish) {
            return;
        }
    }

    std::lock_guard<std::mutex> lock(decoder_mutex_);
    if (!queueBitstream(data, size)) {
        RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 2000, "Failed to queue bitstream into MMAPI decoder");
    }
}

void MmapiDecodedStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

void MmapiDecodedStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

void MmapiDecodedStreamDelegate::initDecoder()
{
    decoder_ = NvVideoDecoder::createVideoDecoder("insta360_mmapi_decoder", O_NONBLOCK);
    if (!decoder_) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to create NvVideoDecoder");
        return;
    }

    if (decoder_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed subscribing to V4L2_EVENT_RESOLUTION_CHANGE");
        cleanupDecoder();
        return;
    }

    if (decoder_->setOutputPlaneFormat(V4L2_PIX_FMT_H264, MMAPI_CHUNK_SIZE) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed setting MMAPI output plane format");
        cleanupDecoder();
        return;
    }

    if (decoder_->setFrameInputMode(1) < 0) {
        RCLCPP_WARN(owner_->get_logger(), "setFrameInputMode(1) failed; continuing with decoder defaults");
    }

    if (decoder_->setMaxPerfMode(1) < 0) {
        RCLCPP_WARN(owner_->get_logger(), "setMaxPerfMode failed; continuing");
    }

    if (decoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP, 10, true, false) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI output plane buffers");
        cleanupDecoder();
        return;
    }

    output_plane_num_buffers_ = decoder_->output_plane.getNumBuffers();
    output_plane_next_index_ = 0;
    output_plane_queued_ = 0;

    if (decoder_->output_plane.setStreamStatus(true) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed starting MMAPI output plane stream");
        cleanupDecoder();
        return;
    }

    capture_running_.store(true);
    capture_thread_ = std::thread(&MmapiDecodedStreamDelegate::captureLoop, this);
    decoder_ready_ = true;
}

bool MmapiDecodedStreamDelegate::queueBitstream(const uint8_t* data, size_t size)
{
    if (!decoder_) {
        return false;
    }

    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer* buffer = nullptr;

    std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    std::memset(planes, 0, sizeof(planes));
    v4l2_buf.m.planes = planes;

    if (output_plane_queued_ < output_plane_num_buffers_) {
        v4l2_buf.index = output_plane_next_index_++;
        if (output_plane_next_index_ >= output_plane_num_buffers_) {
            output_plane_next_index_ = 0;
        }
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        v4l2_buf.length = MAX_PLANES;
        buffer = decoder_->output_plane.getNthBuffer(v4l2_buf.index);
    } else {
        if (decoder_->output_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 0) < 0) {
            return false;
        }
    }

    if (!buffer || buffer->planes[0].data == nullptr) {
        return false;
    }

    const size_t bytes = std::min<size_t>(size, buffer->planes[0].length);
    std::memcpy(buffer->planes[0].data, data, bytes);

    v4l2_buf.m.planes[0].bytesused = static_cast<uint32_t>(bytes);
    const bool queued = decoder_->output_plane.qBuffer(v4l2_buf, nullptr) >= 0;
    if (queued && output_plane_queued_ < output_plane_num_buffers_) {
        ++output_plane_queued_;
    }

    return queued;
}

bool MmapiDecodedStreamDelegate::setupCapturePlane()
{
    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));

    if (decoder_->capture_plane.getFormat(format) < 0) {
        return false;
    }

    capture_width_ = format.fmt.pix_mp.width;
    capture_height_ = format.fmt.pix_mp.height;

    if (decoder_->setCapturePlaneFormat(
            format.fmt.pix_mp.pixelformat,
            format.fmt.pix_mp.width,
            format.fmt.pix_mp.height) < 0) {
        return false;
    }

    int min_cap_buffers = 0;
    if (decoder_->getMinimumCapturePlaneBuffers(min_cap_buffers) < 0) {
        return false;
    }

    if (decoder_->capture_plane.setupPlane(V4L2_MEMORY_MMAP, static_cast<uint32_t>(min_cap_buffers + 4), true, false) < 0) {
        return false;
    }

    if (decoder_->capture_plane.setStreamStatus(true) < 0) {
        return false;
    }

    for (uint32_t i = 0; i < decoder_->capture_plane.getNumBuffers(); ++i) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes, 0, sizeof(planes));
        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0) {
            return false;
        }
    }

    capture_setup_done_.store(true);
    return true;
}

bool MmapiDecodedStreamDelegate::processCapturedFrame(NvBuffer* buffer)
{
    if (!buffer || capture_width_ == 0 || capture_height_ == 0) {
        return false;
    }

    if (buffer->n_planes < 2 || !buffer->planes[0].data || !buffer->planes[1].data) {
        return false;
    }

    cv::Mat y(
        static_cast<int>(capture_height_),
        static_cast<int>(capture_width_),
        CV_8UC1,
        buffer->planes[0].data,
        buffer->planes[0].fmt.stride);

    cv::Mat uv(
        static_cast<int>(capture_height_ / 2),
        static_cast<int>(capture_width_ / 2),
        CV_8UC2,
        buffer->planes[1].data,
        buffer->planes[1].fmt.stride);

    cv::cvtColorTwoPlane(y, uv, bgr_frame_, cv::COLOR_YUV2BGR_NV12);
    owner_->publishDecodedFrame(bgr_frame_);
    return true;
}

void MmapiDecodedStreamDelegate::captureLoop()
{
    while (capture_running_.load()) {
        if (!resolution_event_seen_) {
            struct v4l2_event ev;
            const int dq_ret = decoder_->dqEvent(ev, 1000);
            if (dq_ret == 0 && ev.type == V4L2_EVENT_RESOLUTION_CHANGE) {
                resolution_event_seen_ = true;
                if (!setupCapturePlane()) {
                    RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI capture plane after resolution event");
                    break;
                }
            } else if (dq_ret < 0 && errno == EINVAL) {
                if (!resolution_event_failed_) {
                    RCLCPP_WARN(owner_->get_logger(), "MMAPI resolution-change events not available (EINVAL). Falling back to format polling.");
                    resolution_event_failed_ = true;
                }
            }

            if (resolution_event_failed_) {
                struct v4l2_format format;
                std::memset(&format, 0, sizeof(format));
                if (decoder_->capture_plane.getFormat(format) == 0 &&
                    format.fmt.pix_mp.width > 0 &&
                    format.fmt.pix_mp.height > 0) {
                    resolution_event_seen_ = true;
                    if (!setupCapturePlane()) {
                        RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI capture plane after format polling");
                        break;
                    }
                }
            }
            continue;
        }

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer* buffer = nullptr;
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes, 0, sizeof(planes));
        v4l2_buf.m.planes = planes;

        if (decoder_->capture_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 0) < 0) {
            continue;
        }

        processCapturedFrame(buffer);

        if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0) {
            RCLCPP_WARN(owner_->get_logger(), "Failed to requeue MMAPI capture buffer");
            break;
        }
    }
}

void MmapiDecodedStreamDelegate::cleanupDecoder()
{
    decoder_ready_ = false;
    capture_running_.store(false);

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    if (decoder_) {
        delete decoder_;
        decoder_ = nullptr;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<MmapiDecodedNode>();
        if (!node->startCamera()) {
            rclcpp::shutdown();
            return -1;
        }

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_mmapi_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
