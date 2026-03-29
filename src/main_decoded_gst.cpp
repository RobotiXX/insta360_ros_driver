#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <glib-object.h>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class GstDecodedNode;

class GstDecodedStreamDelegate : public ins_camera::StreamDelegate {
public:
    GstDecodedStreamDelegate(
        GstDecodedNode* owner,
        GstAppSrc* appsrc,
        int skip_frame,
        bool i_frame_only);
    ~GstDecodedStreamDelegate();

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    bool isIdrFrame(const uint8_t* data, size_t size) const;

    GstDecodedNode* owner_;
    GstAppSrc* appsrc_;

    int skip_frame_ = 0;
    bool i_frame_only_ = false;
    bool streaming_ = false;
    bool wait_for_first_idr_ = true;

    std::mutex push_mutex_;
};

class GstDecodedNode : public rclcpp::Node {
public:
    GstDecodedNode()
        : Node("insta360_decoded_gst_node")
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);
        declare_parameter("use_hw_decoder", true);
        declare_parameter("low_latency_mode", true);

        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();
        use_hw_decoder_ = get_parameter("use_hw_decoder").as_bool();
        low_latency_mode_ = get_parameter("low_latency_mode").as_bool();

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS());
    }

    ~GstDecodedNode() override
    {
        if (cam_) {
            cam_->Close();
        }

        std::lock_guard<std::mutex> lock(gst_mutex_);
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_ = nullptr;
            appsink_ = nullptr;
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

        stream_delegate_ = std::make_shared<GstDecodedStreamDelegate>(
            this,
            GST_APP_SRC(appsrc_),
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

        RCLCPP_INFO(get_logger(), "Started integrated GStreamer decode node.");
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
        auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_image.toImageMsg(*out_msg);
        image_pub_->publish(std::move(out_msg));

        ++decoded_frame_count_;
        const auto now_tp = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - fps_window_start_).count();
        if (elapsed >= 2000) {
            const double fps = (1000.0 * static_cast<double>(decoded_frame_count_)) / static_cast<double>(elapsed);
            RCLCPP_INFO(get_logger(), "GST decoded output fps: %.2f", fps);
            decoded_frame_count_ = 0;
            fps_window_start_ = now_tp;
        }
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
        auto* self = static_cast<GstDecodedNode*>(user_data);
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
            if (skip_frame_ > 0) {
                const bool should_publish = (decoded_frame_skip_counter_++ % (skip_frame_ + 1) == 0);
                if (should_publish) {
                    publishDecodedFrame(wrapped);
                }
            } else {
                publishDecodedFrame(wrapped);
            }
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
        GstElement* queue = gst_element_factory_make("queue", "decode_queue");
        GstElement* decoder = gst_element_factory_make(use_hw_decoder_ ? "nvv4l2decoder" : "avdec_h264", "decoder");
        GstElement* nvvidconv = gst_element_factory_make("nvvidconv", "nvvidconv");
        GstElement* videoconvert = gst_element_factory_make("videoconvert", "videoconvert");
        GstElement* capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
        appsink_ = gst_element_factory_make("appsink", "sink");

        if (!pipeline_ || !appsrc_ || !parse || !queue || !decoder || !videoconvert || !capsfilter || !appsink_) {
            RCLCPP_ERROR(get_logger(), "Failed to create one or more GStreamer elements.");
            return false;
        }

        const bool using_hw_decoder = use_hw_decoder_;

        if (using_hw_decoder && !nvvidconv) {
            RCLCPP_ERROR(get_logger(), "Hardware decode path requires nvvidconv but element was not created.");
            return false;
        }

        GstCaps* src_caps = gst_caps_new_simple(
            "video/x-h264",
            "stream-format", G_TYPE_STRING, "byte-stream",
            "alignment", G_TYPE_STRING, "nal",
            nullptr);
        g_object_set(G_OBJECT(appsrc_),
            "is-live", TRUE,
            "format", GST_FORMAT_TIME,
            "do-timestamp", TRUE,
            "block", FALSE,
            "max-bytes", static_cast<guint64>(8 * 1024 * 1024),
            "caps", src_caps,
            nullptr);
        gst_caps_unref(src_caps);

        g_object_set(G_OBJECT(queue),
            "leaky", 2,
            "max-size-buffers", 4,
            "max-size-bytes", 0,
            "max-size-time", static_cast<guint64>(0),
            nullptr);

        g_object_set(G_OBJECT(parse), "disable-passthrough", TRUE, nullptr);

        if (using_hw_decoder) {
            g_object_set(G_OBJECT(decoder), "enable-max-performance", TRUE, nullptr);
            if (low_latency_mode_) {
                GParamSpec* disable_dpb_spec = g_object_class_find_property(G_OBJECT_GET_CLASS(decoder), "disable-dpb");
                if (disable_dpb_spec) {
                    g_object_set(G_OBJECT(decoder), "disable-dpb", TRUE, nullptr);
                }

                GParamSpec* num_extra_surfaces_spec = g_object_class_find_property(G_OBJECT_GET_CLASS(decoder), "num-extra-surfaces");
                if (num_extra_surfaces_spec) {
                    g_object_set(G_OBJECT(decoder), "num-extra-surfaces", 0, nullptr);
                }
            }
        }

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
            "enable-last-sample", FALSE,
            nullptr);

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(onNewSampleStatic), this);

        gst_bin_add_many(
            GST_BIN(pipeline_),
            appsrc_,
            parse,
            queue,
            decoder,
            videoconvert,
            capsfilter,
            appsink_,
            nullptr);

        bool linked = false;
        if (using_hw_decoder) {
            gst_bin_add(GST_BIN(pipeline_), nvvidconv);
            linked = gst_element_link_many(appsrc_, parse, queue, decoder, nvvidconv, videoconvert, capsfilter, appsink_, nullptr);
        } else {
            linked = gst_element_link_many(appsrc_, parse, queue, decoder, videoconvert, capsfilter, appsink_, nullptr);
        }

        if (!linked) {
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

    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    GstElement* pipeline_ = nullptr;
    GstElement* appsrc_ = nullptr;
    GstElement* appsink_ = nullptr;

    int skip_frame_ = 0;
    bool i_frame_only_ = false;
    bool use_hw_decoder_ = true;
    bool low_latency_mode_ = true;

    std::chrono::steady_clock::time_point fps_window_start_ = std::chrono::steady_clock::now();
    uint32_t decoded_frame_count_ = 0;
    uint32_t decoded_frame_skip_counter_ = 0;

    std::mutex gst_mutex_;

    friend class GstDecodedStreamDelegate;
};

GstDecodedStreamDelegate::GstDecodedStreamDelegate(
    GstDecodedNode* owner,
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

GstDecodedStreamDelegate::~GstDecodedStreamDelegate()
{
    if (appsrc_) {
        gst_app_src_end_of_stream(appsrc_);
    }
}

void GstDecodedStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

bool GstDecodedStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
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

void GstDecodedStreamDelegate::OnVideoData(
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

    // Start only once we see an IDR to avoid feeding the decoder mid-GOP,
    // which can look like block patches/corruption.
    if (wait_for_first_idr_) {
        if (!isIdrFrame(data, size)) {
            return;
        }
        wait_for_first_idr_ = false;
        RCLCPP_INFO(owner_->get_logger(), "Received first IDR frame, starting GST decode feed.");
    }

    if (i_frame_only_ && !isIdrFrame(data, size)) {
        return;
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

void GstDecodedStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

void GstDecodedStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<GstDecodedNode>();
        if (!node->startCamera()) {
            rclcpp::shutdown();
            return -1;
        }

        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_gst_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
