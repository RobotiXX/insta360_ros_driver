#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class GstDecodedEquirectNode;

class GstDecodedEquirectStreamDelegate : public ins_camera::StreamDelegate {
public:
    GstDecodedEquirectStreamDelegate(
        GstDecodedEquirectNode* owner,
        GstAppSrc* appsrc,
        int skip_frame,
        bool i_frame_only);
    ~GstDecodedEquirectStreamDelegate() override;

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    bool isIdrFrame(const uint8_t* data, size_t size) const;

    GstDecodedEquirectNode* owner_;
    GstAppSrc* appsrc_;

    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool streaming_ = false;

    std::mutex push_mutex_;
};

class GstDecodedEquirectNode : public rclcpp::Node {
public:
    GstDecodedEquirectNode()
        : Node("insta360_decoded_equirect_gst_node"),
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
            std::bind(&GstDecodedEquirectNode::parametersCallback, this, std::placeholders::_1));
    }

    ~GstDecodedEquirectNode() override
    {
        if (cam_) {
            cam_->Close();
        }

        std::lock_guard<std::mutex> lock(gst_mutex_);
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

    bool startCamera()
    {
        if (!startDecoderPipeline()) {
            RCLCPP_ERROR(get_logger(), "Failed to start GStreamer decoder pipeline.");
            return false;
        }

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
            std::make_shared<GstDecodedEquirectStreamDelegate>(this, GST_APP_SRC(appsrc_), skip_frame_, i_frame_only_);
        stream_delegate_ = delegate;
        cam_->SetStreamDelegate(delegate);

        uint64_t utc_time = static_cast<uint64_t>(time(NULL));
        cam_->SyncLocalTimeToCamera(utc_time, 0);

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_1920_960P30;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Started integrated GStreamer decode + C++ equirectangular node.");
        return true;
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
    static GstFlowReturn onNewSampleStatic(GstAppSink* sink, gpointer user_data)
    {
        auto* self = static_cast<GstDecodedEquirectNode*>(user_data);
        return self->onNewSample(sink);
    }

    GstFlowReturn onNewSample(GstAppSink* sink)
    {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            return GST_FLOW_ERROR;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        if (!buffer || !caps) {
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        GstVideoInfo info;
        if (!gst_video_info_from_caps(&info, caps)) {
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        const int width = GST_VIDEO_INFO_WIDTH(&info);
        const int height = GST_VIDEO_INFO_HEIGHT(&info);
        const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);

        if (width <= 0 || height <= 0 || stride <= 0) {
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }

        if (static_cast<int>(map.size) >= stride * height) {
            cv::Mat wrapped(height, width, CV_8UC3, const_cast<guint8*>(map.data), stride);
            cv::Mat frame = wrapped.clone();
            processDecodedFrame(frame);
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    bool startDecoderPipeline()
    {
        std::lock_guard<std::mutex> lock(gst_mutex_);

        if (pipeline_) {
            return true;
        }

        static bool gst_initialized = false;
        if (!gst_initialized) {
            gst_init(nullptr, nullptr);
            gst_initialized = true;
        }

        pipeline_ = gst_pipeline_new("insta360_decode_pipeline");
        appsrc_ = gst_element_factory_make("appsrc", "src");
        GstElement* parse = gst_element_factory_make("h264parse", "parse");
        GstElement* decoder = gst_element_factory_make("nvv4l2decoder", "decoder");
        GstElement* nvvidconv = gst_element_factory_make("nvvidconv", "nvvidconv");
        GstElement* videoconvert = gst_element_factory_make("videoconvert", "videoconvert");
        GstElement* capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
        appsink_ = gst_element_factory_make("appsink", "sink");

        if (!pipeline_ || !appsrc_ || !parse || !decoder || !nvvidconv || !videoconvert || !capsfilter || !appsink_) {
            RCLCPP_ERROR(get_logger(), "Failed to create one or more GStreamer elements.");
            return false;
        }

        GstCaps* src_caps = gst_caps_new_simple(
            "video/x-h264",
            "stream-format", G_TYPE_STRING, "byte-stream",
            "alignment", G_TYPE_STRING, "au",
            nullptr);
        g_object_set(G_OBJECT(appsrc_),
            "is-live", TRUE,
            "format", GST_FORMAT_TIME,
            "do-timestamp", TRUE,
            "block", FALSE,
            "caps", src_caps,
            nullptr);
        gst_caps_unref(src_caps);

        g_object_set(G_OBJECT(decoder), "enable-max-performance", TRUE, nullptr);

        GstCaps* sink_caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            nullptr);
        g_object_set(G_OBJECT(capsfilter), "caps", sink_caps, nullptr);
        gst_caps_unref(sink_caps);

        g_object_set(G_OBJECT(appsink_),
            "emit-signals", TRUE,
            "sync", FALSE,
            "drop", TRUE,
            "max-buffers", 1,
            nullptr);

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(onNewSampleStatic), this);

        gst_bin_add_many(
            GST_BIN(pipeline_),
            appsrc_,
            parse,
            decoder,
            nvvidconv,
            videoconvert,
            capsfilter,
            appsink_,
            nullptr);

        if (!gst_element_link_many(appsrc_, parse, decoder, nvvidconv, videoconvert, capsfilter, appsink_, nullptr)) {
            RCLCPP_ERROR(get_logger(), "Failed to link GStreamer decoder pipeline.");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_ = nullptr;
            appsink_ = nullptr;
            return false;
        }

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_ERROR(get_logger(), "Failed to set GStreamer pipeline to PLAYING.");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_ = nullptr;
            appsink_ = nullptr;
            return false;
        }

        return true;
    }

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

    GstElement* pipeline_ = nullptr;
    GstElement* appsrc_ = nullptr;
    GstElement* appsink_ = nullptr;

    std::mutex process_mutex_;
    std::mutex gst_mutex_;

    friend class GstDecodedEquirectStreamDelegate;
};

GstDecodedEquirectStreamDelegate::GstDecodedEquirectStreamDelegate(
    GstDecodedEquirectNode* owner,
    GstAppSrc* appsrc,
    int skip_frame,
    bool i_frame_only)
    : owner_(owner),
      appsrc_(appsrc),
      skip_frame_(skip_frame),
      i_frame_only_(i_frame_only),
      streaming_(appsrc != nullptr)
{
}

GstDecodedEquirectStreamDelegate::~GstDecodedEquirectStreamDelegate()
{
    if (appsrc_) {
        gst_app_src_end_of_stream(appsrc_);
    }
}

void GstDecodedEquirectStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

bool GstDecodedEquirectStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
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

void GstDecodedEquirectStreamDelegate::OnVideoData(
    const uint8_t* data,
    size_t size,
    int64_t timestamp,
    uint8_t streamType,
    int stream_index)
{
    (void)timestamp;
    (void)streamType;

    if (!streaming_ || !appsrc_ || stream_index != 0 || size == 0) {
        return;
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

    std::lock_guard<std::mutex> lock(push_mutex_);

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) {
        return;
    }

    gst_buffer_fill(buffer, 0, data, size);
    const GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buffer);
    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
        RCLCPP_WARN(owner_->get_logger(), "gst_app_src_push_buffer failed: %d", ret);
    }
}

void GstDecodedEquirectStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

void GstDecodedEquirectStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<GstDecodedEquirectNode>();
        if (!node->startCamera()) {
            rclcpp::shutdown();
            return -1;
        }

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_equirect_gst_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
