#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <fcntl.h>

#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "NvVideoDecoder.h"
#include "NvBuffer.h"

#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define MMAPI_CHUNK_SIZE (4 * 1024 * 1024)

class DecodedEquirectNode;

class DecodedEquirectStreamDelegate : public ins_camera::StreamDelegate {
public:
    DecodedEquirectStreamDelegate(DecodedEquirectNode* owner, int skip_frame, bool i_frame_only);
    ~DecodedEquirectStreamDelegate();

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

    DecodedEquirectNode* owner_;

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

class DecodedEquirectNode : public rclcpp::Node {
public:
    DecodedEquirectNode()
        : Node("insta360_decoded_equirect_mmapi_node"),
          maps_initialized_(false),
          params_changed_(true),
          img_height_(0),
          img_width_(0)
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);

        declare_parameter("cx_offset", 0.0);
        declare_parameter("cy_offset", 0.0);
        declare_parameter("front_cx_offset", 0.0);
        declare_parameter("front_cy_offset", 0.0);
        declare_parameter("back_cx_offset", 0.0);
        declare_parameter("back_cy_offset", 0.0);
        declare_parameter("crop_size", 960);
        declare_parameter("translation", std::vector<double>{0.0, 0.0, -0.105});
        declare_parameter("rotation_deg", std::vector<double>{-0.5, 0.0, 1.1});
        declare_parameter("out_width", 1920);
        declare_parameter("out_height", 960);

        loadParameters();
        updateCameraParameters();

        equirect_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/equirectangular/image", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS());

        params_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&DecodedEquirectNode::parametersCallback, this, std::placeholders::_1));
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
            std::make_shared<DecodedEquirectStreamDelegate>(this, skip_frame_, i_frame_only_);
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

        RCLCPP_INFO(get_logger(), "Started integrated Jetson MMAPI decode + C++ equirectangular node.");
        return true;
    }

    ~DecodedEquirectNode() override
    {
        if (cam_) {
            cam_->Close();
        }
    }

    void processDecodedFrame(const cv::Mat& decoded_bgr)
    {
        std::lock_guard<std::mutex> lock(process_mutex_);

        if (decoded_bgr.empty()) {
            return;
        }

        cv::Mat dual_fisheye_img;
        cv::cvtColor(decoded_bgr, dual_fisheye_img, cv::COLOR_BGR2RGB);

        int img_height = dual_fisheye_img.rows;
        int img_width_full = dual_fisheye_img.cols;
        int midpoint = img_width_full / 2;

        cv::Mat front_img_full = dual_fisheye_img(cv::Rect(midpoint, 0, midpoint, img_height));
        cv::Mat back_img_full = dual_fisheye_img(cv::Rect(0, 0, midpoint, img_height));

        cv::Mat front_img;
        cv::Mat back_img;
        int orig_h = front_img_full.rows;
        int orig_w = front_img_full.cols;

        if (orig_h != crop_size_ || orig_w != crop_size_) {
            int y_start = (orig_h - crop_size_) / 2;
            int x_start = (orig_w - crop_size_) / 2;

            if (y_start >= 0 && x_start >= 0 &&
                y_start + crop_size_ <= orig_h &&
                x_start + crop_size_ <= orig_w) {
                front_img = front_img_full(cv::Rect(x_start, y_start, crop_size_, crop_size_));
                back_img = back_img_full(cv::Rect(x_start, y_start, crop_size_, crop_size_));
            } else {
                front_img = front_img_full;
                back_img = back_img_full;
            }
        } else {
            front_img = front_img_full;
            back_img = back_img_full;
        }

        if (!maps_initialized_ || params_changed_ ||
            front_img.rows != img_height_ || front_img.cols != img_width_) {
            initMapping(front_img.rows, front_img.cols);
            params_changed_ = false;
        }

        cv::Mat equirect_img = createEquirectangular(front_img, back_img);

        auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera_frame";
        cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::RGB8, equirect_img);
        cv_image.toImageMsg(*out_msg);
        equirect_pub_->publish(std::move(out_msg));
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
    void loadParameters()
    {
        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();

        cx_offset_ = get_parameter("cx_offset").as_double();
        cy_offset_ = get_parameter("cy_offset").as_double();
        front_cx_offset_ = get_parameter("front_cx_offset").as_double();
        front_cy_offset_ = get_parameter("front_cy_offset").as_double();
        back_cx_offset_ = get_parameter("back_cx_offset").as_double();
        back_cy_offset_ = get_parameter("back_cy_offset").as_double();
        crop_size_ = get_parameter("crop_size").as_int();
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();

        auto translation = get_parameter("translation").as_double_array();
        tx_ = translation[0];
        ty_ = translation[1];
        tz_ = translation[2];

        auto rotation_deg = get_parameter("rotation_deg").as_double_array();
        roll_ = rotation_deg[0] * M_PI / 180.0;
        pitch_ = rotation_deg[1] * M_PI / 180.0;
        yaw_ = rotation_deg[2] * M_PI / 180.0;
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        bool update_needed = false;
        for (const auto& param : parameters) {
            const auto& name = param.get_name();
            if (name == "skip_frame" || name == "i_frame_only" ||
                name == "cx_offset" || name == "cy_offset" ||
                name == "front_cx_offset" || name == "front_cy_offset" ||
                name == "back_cx_offset" || name == "back_cy_offset" ||
                name == "crop_size" || name == "translation" ||
                name == "rotation_deg" || name == "out_width" || name == "out_height") {
                update_needed = true;
                break;
            }
        }

        if (update_needed) {
            std::lock_guard<std::mutex> lock(process_mutex_);
            loadParameters();
            updateCameraParameters();
            params_changed_ = true;
            maps_initialized_ = false;
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void updateCameraParameters()
    {
        cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
            1.0, 0.0, 0.0,
            0.0, cos(roll_), -sin(roll_),
            0.0, sin(roll_), cos(roll_));

        cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
            cos(pitch_), 0.0, sin(pitch_),
            0.0, 1.0, 0.0,
            -sin(pitch_), 0.0, cos(pitch_));

        cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
            cos(yaw_), -sin(yaw_), 0.0,
            sin(yaw_), cos(yaw_), 0.0,
            0.0, 0.0, 1.0);

        back_to_front_rotation_ = Rz * Ry * Rx;
        back_to_front_translation_ = cv::Vec3d(tx_, ty_, tz_);
    }

    void initMapping(int img_height, int img_width)
    {
        img_height_ = img_height;
        img_width_ = img_width;

        double base_cx = img_width / 2.0 + cx_offset_;
        double base_cy = img_height / 2.0 + cy_offset_;
        double front_cx = base_cx + front_cx_offset_;
        double front_cy = base_cy + front_cy_offset_;
        double back_cx = base_cx + back_cx_offset_;
        double back_cy = base_cy + back_cy_offset_;

        cv::Mat x_grid, y_grid;
        cv::Mat x_range = cv::Mat::zeros(1, out_width_, CV_32F);
        cv::Mat y_range = cv::Mat::zeros(out_height_, 1, CV_32F);

        for (int i = 0; i < out_width_; ++i) {
            x_range.at<float>(0, i) = static_cast<float>(i);
        }
        for (int i = 0; i < out_height_; ++i) {
            y_range.at<float>(i, 0) = static_cast<float>(i);
        }

        cv::repeat(x_range, out_height_, 1, x_grid);
        cv::repeat(y_range, 1, out_width_, y_grid);

        cv::Mat longitude = (x_grid / static_cast<float>(out_width_)) * 2 * M_PI - M_PI;
        cv::Mat latitude = (y_grid / static_cast<float>(out_height_)) * M_PI - M_PI / 2;

        cv::Mat X, Y, Z;
        cv::Mat cos_lat, sin_lat, cos_lon, sin_lon;
        cos_lat = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        sin_lat = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        cos_lon = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        sin_lon = cv::Mat::zeros(out_height_, out_width_, CV_32F);

        for (int y = 0; y < out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                float lat = latitude.at<float>(y, x);
                float lon = longitude.at<float>(y, x);
                cos_lat.at<float>(y, x) = cos(lat);
                sin_lat.at<float>(y, x) = sin(lat);
                cos_lon.at<float>(y, x) = cos(lon);
                sin_lon.at<float>(y, x) = sin(lon);
            }
        }

        X = cos_lat.mul(sin_lon);
        Y = sin_lat;
        Z = cos_lat.mul(cos_lon);

        front_mask_ = Z >= 0;
        back_mask_ = Z < 0;

        front_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        front_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        back_map_x_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);
        back_map_y_ = cv::Mat::zeros(out_height_, out_width_, CV_32F);

        for (int y = 0; y < out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                if (front_mask_.at<uchar>(y, x)) {
                    float X_val = X.at<float>(y, x);
                    float Y_val = Y.at<float>(y, x);
                    float Z_val = Z.at<float>(y, x);

                    float r = sqrt(X_val * X_val + Y_val * Y_val);
                    if (r < 1e-6f) {
                        r = 1e-6f;
                    }

                    float theta = atan2(r, fabs(Z_val));
                    float r_fisheye = 2 * theta / M_PI * (img_width / 2.0f);

                    front_map_x_.at<float>(y, x) = front_cx + X_val / r * r_fisheye;
                    front_map_y_.at<float>(y, x) = front_cy + Y_val / r * r_fisheye;
                }
            }
        }

        for (int y = 0; y < out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                if (back_mask_.at<uchar>(y, x)) {
                    cv::Vec3d point(X.at<float>(y, x), Y.at<float>(y, x), Z.at<float>(y, x));
                    cv::Mat transformed = back_to_front_rotation_ * cv::Mat(point) + cv::Mat(back_to_front_translation_);

                    float X_back = -transformed.at<double>(0);
                    float Y_back = transformed.at<double>(1);
                    float Z_back = transformed.at<double>(2);

                    float r = sqrt(X_back * X_back + Y_back * Y_back);
                    if (r < 1e-6f) {
                        r = 1e-6f;
                    }

                    float theta = atan2(r, fabs(Z_back));
                    float r_fisheye = 2 * theta / M_PI * (img_width / 2.0f);

                    back_map_x_.at<float>(y, x) = back_cx + X_back / r * r_fisheye;
                    back_map_y_.at<float>(y, x) = back_cy + Y_back / r * r_fisheye;
                }
            }
        }

        maps_initialized_ = true;
    }

    cv::Mat createEquirectangular(const cv::Mat& front_img, const cv::Mat& back_img)
    {
        cv::Mat front_result, back_result;
        cv::remap(front_img, front_result, front_map_x_, front_map_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(back_img, back_result, back_map_x_, back_map_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::Mat equirect = cv::Mat::zeros(out_height_, out_width_, CV_8UC3);
        cv::Mat front = front_mask_;
        cv::Mat back = ~front_mask_;
        front_result.copyTo(equirect, front);
        back_result.copyTo(equirect, back);
        return equirect;
    }

    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr equirect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    bool maps_initialized_;
    bool params_changed_;
    int img_height_;
    int img_width_;

    int skip_frame_;
    bool i_frame_only_;

    double cx_offset_;
    double cy_offset_;
    double front_cx_offset_;
    double front_cy_offset_;
    double back_cx_offset_;
    double back_cy_offset_;
    int crop_size_;
    double tx_, ty_, tz_;
    double roll_, pitch_, yaw_;
    int out_width_;
    int out_height_;

    cv::Mat back_to_front_rotation_;
    cv::Vec3d back_to_front_translation_;
    cv::Mat front_map_x_, front_map_y_;
    cv::Mat back_map_x_, back_map_y_;
    cv::Mat front_mask_, back_mask_;

    std::mutex process_mutex_;

    friend class DecodedEquirectStreamDelegate;
};

DecodedEquirectStreamDelegate::DecodedEquirectStreamDelegate(
    DecodedEquirectNode* owner,
    int skip_frame,
    bool i_frame_only)
    : owner_(owner),
      skip_frame_(skip_frame),
      i_frame_only_(i_frame_only)
{
    initDecoder();
}

DecodedEquirectStreamDelegate::~DecodedEquirectStreamDelegate()
{
    cleanupDecoder();
}

void DecodedEquirectStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

bool DecodedEquirectStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
{
    for (size_t i = 0; i + 4 < size; ++i) {
        const bool start3 = (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01);
        const bool start4 = (i + 5 < size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00 && data[i + 3] == 0x01);
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

void DecodedEquirectStreamDelegate::OnVideoData(
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

    // MMAPI decode startup is more stable when we begin from an IDR frame.
    if (wait_for_first_idr_) {
        if (!isIdrFrame(data, size)) {
            return;
        }
        wait_for_first_idr_ = false;
        RCLCPP_INFO(owner_->get_logger(), "Received first IDR frame, starting MMAPI decode feed.");
    }

    if (skip_frame_ > 0 && !i_frame_only_) {
        bool should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
        if (!should_publish) {
            return;
        }
    }

    std::lock_guard<std::mutex> lock(decoder_mutex_);
    if (!queueBitstream(data, size)) {
        RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 2000, "Failed to queue bitstream into MMAPI decoder");
    }
}

void DecodedEquirectStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

void DecodedEquirectStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

void DecodedEquirectStreamDelegate::initDecoder()
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

    // Input is a byte-stream bitstream chunk from camera callback.
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
    capture_thread_ = std::thread(&DecodedEquirectStreamDelegate::captureLoop, this);
    decoder_ready_ = true;
}

bool DecodedEquirectStreamDelegate::queueBitstream(const uint8_t* data, size_t size)
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

    // Before all output-plane buffers are queued once, grab a free MMAP buffer
    // directly by index. After that, recycle buffers via dq/q.
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

bool DecodedEquirectStreamDelegate::setupCapturePlane()
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

bool DecodedEquirectStreamDelegate::processCapturedFrame(NvBuffer* buffer)
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
    owner_->processDecodedFrame(bgr_frame_);
    return true;
}

void DecodedEquirectStreamDelegate::captureLoop()
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

void DecodedEquirectStreamDelegate::cleanupDecoder()
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
        auto node = std::make_shared<DecodedEquirectNode>();
        if (!node->startCamera()) {
            rclcpp::shutdown();
            return -1;
        }

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_equirect_mmapi_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
