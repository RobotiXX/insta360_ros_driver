/**
 * @file main_decoded_gstreamer_opencv.cpp
 * @brief GStreamer + CUDA-enabled OpenCV H.264 decoder node for Insta360 camera
 * 
 * PURPOSE:
 * Alternative decoder implementation using GStreamer pipeline instead of NVIDIA MMAPI.
 * This approach provides:
 * - Simple, portable GStreamer pipeline for H.264→NVIDIA GPU decoding
 * - CUDA-enabled OpenCV for GPU-accelerated post-processing
 * - Flexible color space conversion using OpenCV GPU routines
 * - Easy reconfiguration via GStreamer properties
 * 
 * ADVANTAGES vs MMAPI:
 * - Simpler API (GStreamer abstracts V4L2 complexity)
 * - More portable (not Jetson-specific)
 * - Built-in logging and debugging
 * - Can easily add inference/processing via OpenCV
 * 
 * REQUIREMENTS:
 * - GStreamer 1.0 with plugins: h264parse, omxh264dec or nvh264dec, filesink
 * - OpenCV 4.5+ compiled with CUDA support
 * - NVIDIA Jetson with CUDA toolkit
 */

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <queue>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

/// ===== GSTREAMER CONFIGURATION =====
/// Buffer size for H.264 bitstream chunks (larger = fewer context switches)
#define GSTREAMER_BUFFER_SIZE (1024 * 1024)  // 1 MB chunks
/// Queue size for frame buffer pool (prevents memory exhaustion)
#define GSTREAMER_QUEUE_DEPTH 10

class GStreamerDecodedNode;

/**
 * @class GStreamerDecodedStreamDelegate
 * @brief Stream callback for Insta360 camera with GStreamer H.264 decoding
 * 
 * Receives H.264 video/IMU from camera, feeds into GStreamer pipeline for GPU decoding,
 * retrieves decoded frames via AppSink, and publishes to ROS.
 * 
 * ARCHITECTURE:
 * Camera → OnVideoData() → [H.264 bitstream] → GStreamer appsrc
 *    ↓
 * [GStreamer Pipeline: h264parse ! nvh264dec ! appsink]
 *    ↓
 * [GPU-decoded BGR/I420 frames] → appsink pull → OpenCV processing
 *    ↓
 * publishDecodedFrame() → ROS topic
 */
class GStreamerDecodedStreamDelegate : public ins_camera::StreamDelegate {
public:
    GStreamerDecodedStreamDelegate(GStreamerDecodedNode* owner, int skip_frame, bool i_frame_only);
    ~GStreamerDecodedStreamDelegate();

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    // ===== GSTREAMER PIPELINE SETUP & MANAGEMENT =====
    bool initGStreamerPipeline();
    void cleanupGStreamerPipeline();
    bool pushBitstreamBuffer(const uint8_t* data, size_t size);

    // ===== FRAME RETRIEVAL & PROCESSING =====
    void frameRetrievalLoop();
    bool processCapturedFrame(GstSample* sample);

    // ===== UTILITY FUNCTIONS =====
    bool isIdrFrame(const uint8_t* data, size_t size) const;
    static GstFlowReturn onNewSampleCallback(GstAppSink* sink, gpointer user_data);

    GStreamerDecodedNode* owner_;

    // ===== GSTREAMER PIPELINE ELEMENTS =====
    GstElement* pipeline_ = nullptr;          // Main pipeline container
    GstElement* appsrc_ = nullptr;            // Source element (receives bitstream)
    GstElement* h264parse_ = nullptr;         // H.264 NAL unit parser
    GstElement* decoder_ = nullptr;           // NVIDIA hardware decoder (nvh264dec or omxh264dec)
    GstElement* capsfilter_ = nullptr;        // Format converter/filter
    GstElement* appsink_ = nullptr;           // Sink element (receives decoded frames)

    GstCaps* src_caps_ = nullptr;             // Source format caps
    GstCaps* sink_caps_ = nullptr;            // Sink format caps

    // ===== THREADING & SYNCHRONIZATION =====
    std::thread frame_thread_;                // Thread for pulling frames from pipeline
    std::atomic<bool> running_{false};        // Pipeline running flag
    std::atomic<bool> first_frame_received_{false};

    // ===== FRAME BUFFER POOL =====
    std::queue<GstSample*> frame_queue_;      // Queue of processed frames
    std::mutex frame_queue_mutex_;            // Protects frame queue
    std::condition_variable frame_queue_cv_;  // Signals when frame available

    // ===== OPENCV GPU STATE =====
    cv::cuda::GpuMat gpu_yuv_;                // GPU buffer for YUV frame
    cv::cuda::GpuMat gpu_bgr_;                // GPU buffer for BGR conversion result

    // ===== FRAME FILTERING =====
    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool wait_for_first_idr_ = true;

    // ===== RESOLUTION CACHE =====
    uint32_t frame_width_ = 0;
    uint32_t frame_height_ = 0;
};

/**
 * @class GStreamerDecodedNode
 * @brief ROS 2 node for Insta360 camera with GStreamer decoding
 */
class GStreamerDecodedNode : public rclcpp::Node {
public:
    GStreamerDecodedNode()
        : Node("insta360_decoded_gstreamer_node")
    {
        // ===== DECLARE PARAMETERS =====
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);
        declare_parameter("video_bitrate", static_cast<int64_t>(1024 * 1024));
        /// decoder_type: "nvh264dec" (NVIDIA NVDEC) or "omxh264dec" (OMX decoder)
        declare_parameter("decoder_type", "nvh264dec");
        /// enable_gpu_color_convert: If true, use OpenCV GPU for color conversion
        declare_parameter("enable_gpu_color_convert", true);

        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();
        video_bitrate_ = get_parameter("video_bitrate").as_int();
        decoder_type_ = get_parameter("decoder_type").as_string();
        enable_gpu_color_convert_ = get_parameter("enable_gpu_color_convert").as_bool();

        // ===== ROS PUBLISHERS =====
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS());
    }

    ~GStreamerDecodedNode() override
    {
        if (cam_) {
            cam_->Close();
        }
    }

    /**
     * @brief Initialize and start camera streaming
     * @return true if camera started
     */
    bool startCamera()
    {
        // ===== DISCOVER CAMERA DEVICE =====
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(get_logger(), "No available camera devices found.");
            return false;
        }

        // ===== OPEN CAMERA CONNECTION =====
        cam_ = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam_->Open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera.");
            discovery.FreeDeviceDescriptors(list);
            return false;
        }
        discovery.FreeDeviceDescriptors(list);

        // ===== ATTACH STREAM DELEGATE =====
        stream_delegate_ = std::make_shared<GStreamerDecodedStreamDelegate>(
            this,
            skip_frame_,
            i_frame_only_);
        cam_->SetStreamDelegate(stream_delegate_);

        // ===== SYNC CAMERA CLOCK =====
        const uint64_t utc_time = static_cast<uint64_t>(time(nullptr));
        cam_->SyncLocalTimeToCamera(utc_time);

        // ===== CONFIGURE STREAMING =====
        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_2880_1440P30;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = video_bitrate_;
        param.enable_audio = false;
        param.using_lrv = false;

        // ===== START STREAMING =====
        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Started GStreamer+OpenCV CUDA decode node (decoder=%s, gpu_color=%s)",
            decoder_type_.c_str(), enable_gpu_color_convert_ ? "yes" : "no");
        return true;
    }

    /**
     * @brief Publish decoded video frame to ROS
     */
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

    /**
     * @brief Publish IMU gyro/accel data
     */
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
    int64_t video_bitrate_ = 1024 * 1024;
    std::string decoder_type_ = "nvh264dec";
    bool enable_gpu_color_convert_ = true;

    friend class GStreamerDecodedStreamDelegate;
};

// ============================================================================
// GStreamer Callback Functions
// ============================================================================

/**
 * @brief AppSink callback when new sample available
 * @param sink GStreamer sink element
 * @param user_data Pointer to GStreamerDecodedStreamDelegate
 * @return GST_FLOW_OK if handled
 * 
 * Called by GStreamer when decoded frame ready. Queues frame for processing.
 */
static GstFlowReturn onNewSampleCallback(GstAppSink* sink, gpointer user_data)
{
    auto* delegate = reinterpret_cast<GStreamerDecodedStreamDelegate*>(user_data);
    if (!delegate) {
        return GST_FLOW_ERROR;
    }

    // ===== RETRIEVE NEW SAMPLE FROM SINK =====
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
        return GST_FLOW_ERROR;
    }

    // ===== QUEUE FOR PROCESSING =====
    {
        std::lock_guard<std::mutex> lock(delegate->frame_queue_mutex_);
        if (delegate->frame_queue_.size() < GSTREAMER_QUEUE_DEPTH) {
            delegate->frame_queue_.push(sample);
            delegate->frame_queue_cv_.notify_one();
        } else {
            // ===== QUEUE FULL - DROP FRAME =====
            gst_sample_unref(sample);
        }
    }

    return GST_FLOW_OK;
}

// ============================================================================
// StreamDelegate Implementation
// ============================================================================

GStreamerDecodedStreamDelegate::GStreamerDecodedStreamDelegate(
    GStreamerDecodedNode* owner,
    int skip_frame,
    bool i_frame_only)
    : owner_(owner),
      skip_frame_(skip_frame),
      i_frame_only_(i_frame_only)
{
    if (!initGStreamerPipeline()) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to initialize GStreamer pipeline");
        return;
    }

    // ===== START FRAME RETRIEVAL THREAD =====
    running_.store(true);
    frame_thread_ = std::thread(&GStreamerDecodedStreamDelegate::frameRetrievalLoop, this);
}

GStreamerDecodedStreamDelegate::~GStreamerDecodedStreamDelegate()
{
    cleanupGStreamerPipeline();
}

/**
 * @brief Initialize GStreamer pipeline
 * @return true if pipeline created and running
 * 
 * Creates pipeline:
 * appsrc ! h264parse ! nvh264dec ! appsink
 * 
 * - appsrc: Receives H.264 bitstream from camera
 * - h264parse: Parses H.264 NAL units, handles frame boundaries
 * - nvh264dec: NVIDIA HW decoder (or omxh264dec fallback)
 * - appsink: Outputs decoded RGB/YUV frames for OpenCV processing
 */
bool GStreamerDecodedStreamDelegate::initGStreamerPipeline()
{
    GError* error = nullptr;

    // ===== PARSE PIPELINE DESCRIPTION =====
    // Elements connected: appsrc ! h264parse ! [decoder] ! appsink
    std::string decoder_name = owner_->decoder_type_;
    std::string pipeline_str = "appsrc name=src ! h264parse ! " + decoder_name + 
                                " ! queue ! appsink name=sink";

    RCLCPP_INFO(owner_->get_logger(), "Creating GStreamer pipeline: %s", pipeline_str.c_str());

    // ===== CREATE PIPELINE FROM DESCRIPTION =====
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline_) {
        if (error) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to create pipeline: %s", error->message);
            g_error_free(error);
        }
        return false;
    }

    // ===== GET REFERENCES TO PIPELINE ELEMENTS =====
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    if (!appsrc_ || !appsink_) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to get pipeline elements");
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // ===== CONFIGURE APPSRC ELEMENT (INPUT) =====
    // This receives H.264 bitstream data pushed from OnVideoData
    GstCaps* appsrc_caps = gst_caps_new_simple("video/x-h264",
        "stream-format", G_TYPE_STRING, "byte-stream",
        nullptr);

    g_object_set(G_OBJECT(appsrc_),
        "caps", appsrc_caps,
        "format", GST_FORMAT_BYTES,
        "block", TRUE,  // Block on full queue
        nullptr);

    gst_caps_unref(appsrc_caps);

    // ===== CONFIGURE APPSINK ELEMENT (OUTPUT) =====
    // This delivers decoded frames to frame_queue via onNewSampleCallback
    GstCaps* appsink_caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGRx",  // Request BGR format for OpenCV
        nullptr);

    g_object_set(G_OBJECT(appsink_),
        "caps", appsink_caps,
        "emit-signals", TRUE,
        "max-buffers", GSTREAMER_QUEUE_DEPTH,
        nullptr);

    // ===== CONNECT APPSINK SIGNAL =====
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(onNewSampleCallback), this);

    gst_caps_unref(appsink_caps);

    // ===== SET PIPELINE TO PLAYING STATE =====
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to set pipeline to PLAYING state");
        gst_object_unref(appsrc_);
        gst_object_unref(appsink_);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    RCLCPP_INFO(owner_->get_logger(), "GStreamer pipeline initialized successfully");
    return true;
}

/**
 * @brief Cleanup GStreamer pipeline
 */
void GStreamerDecodedStreamDelegate::cleanupGStreamerPipeline()
{
    // ===== STOP PIPELINE =====
    running_.store(false);

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }

    // ===== JOIN FRAME THREAD =====
    if (frame_thread_.joinable()) {
        frame_thread_.join();
    }

    // ===== DRAIN AND CLEANUP FRAME QUEUE =====
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        while (!frame_queue_.empty()) {
            GstSample* sample = frame_queue_.front();
            frame_queue_.pop();
            gst_sample_unref(sample);
        }
    }

    // ===== UNREFERENCE PIPELINE ELEMENTS =====
    if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
    if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

/**
 * @brief Push H.264 bitstream buffer into GStreamer pipeline
 * @param data H.264 NAL unit(s)
 * @param size Size in bytes
 * @return true if pushed successfully
 * 
 * Wraps data in GStreamer buffer and pushes into appsrc element.
 */
bool GStreamerDecodedStreamDelegate::pushBitstreamBuffer(const uint8_t* data, size_t size)
{
    if (!appsrc_ || !data || size == 0) {
        return false;
    }

    // ===== ALLOCATE GSTREAMER BUFFER =====
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) {
        return false;
    }

    // ===== COPY H.264 DATA INTO GSTREAMER BUFFER =====
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        return false;
    }

    std::memcpy(map.data, data, size);
    gst_buffer_unmap(buffer, &map);

    // ===== PUSH BUFFER INTO APPSRC =====
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);

    // ===== RETURN SUCCESS/FAILURE =====
    return (ret == GST_FLOW_OK);
}

/**
 * @brief Check if NAL unit is IDR frame
 * @param data H.264 bitstream
 * @param size Size of data
 * @return true if IDR frame found
 */
bool GStreamerDecodedStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
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
        if (nal_type == 5) {  // NAL type 5 = IDR
            return true;
        }
    }

    return false;
}

/**
 * @brief Audio data callback (not used)
 */
void GStreamerDecodedStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

/**
 * @brief Video data callback - queues H.264 frames
 * @param data H.264 bitstream
 * @param size Size in bytes
 * @param timestamp Camera capture timestamp
 * @param streamType Stream type
 * @param stream_index Stream index (0 = main video)
 * 
 * Filters frames based on skip_frame/i_frame_only, then pushes to GStreamer.
 */
void GStreamerDecodedStreamDelegate::OnVideoData(
    const uint8_t* data,
    size_t size,
    int64_t timestamp,
    uint8_t streamType,
    int stream_index)
{
    (void)timestamp;
    (void)streamType;

    // ===== REJECT IF: WRONG STREAM, EMPTY DATA, OR PIPELINE NOT READY =====
    if (stream_index != 0 || size == 0 || !pipeline_) {
        return;
    }

    // ===== WAIT FOR FIRST IDR FRAME =====
    if (wait_for_first_idr_) {
        if (!isIdrFrame(data, size)) {
            return;
        }
        wait_for_first_idr_ = false;
        RCLCPP_INFO(owner_->get_logger(), "Received first IDR frame, starting GStreamer decode feed.");
    }

    // ===== FILTER: I-FRAME ONLY MODE =====
    if (i_frame_only_ && !isIdrFrame(data, size)) {
        return;
    }

    // ===== FILTER: FRAME SKIPPING =====
    if (skip_frame_ > 0 && !i_frame_only_) {
        const bool should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
        if (!should_publish) {
            return;
        }
    }

    // ===== PUSH TO GSTREAMER PIPELINE =====
    if (!pushBitstreamBuffer(data, size)) {
        RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 2000,
            "Failed to push bitstream to GStreamer pipeline");
    }
}

/**
 * @brief IMU data callback
 */
void GStreamerDecodedStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

/**
 * @brief Exposure data callback (not used)
 */
void GStreamerDecodedStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

/**
 * @brief Process decoded frame from GStreamer
 * @param sample GStreamer sample containing decoded video frame
 * @return true if processed
 * 
 * Extracts GStreamer buffer, converts to OpenCV Mat (potentially GPU-accelerated),
 * and publishes to ROS.
 */
bool GStreamerDecodedStreamDelegate::processCapturedFrame(GstSample* sample)
{
    if (!sample) {
        return false;
    }

    // ===== EXTRACT BUFFER AND CAPS FROM SAMPLE =====
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);

    if (!buffer || !caps) {
        return false;
    }

    // ===== EXTRACT RESOLUTION FROM CAPS =====
    GstStructure* structure = gst_caps_get_structure(caps, 0);
    gint width = 0, height = 0;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    if (width == 0 || height == 0) {
        return false;
    }

    // ===== MAP BUFFER FOR CPU ACCESS =====
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        return false;
    }

    // ===== CONVERT GSTREAMER BUFFER TO OPENCV MAT =====
    // BGRx format: 4 bytes per pixel (BGR + padding)
    cv::Mat bgr_frame(height, width, CV_8UC3);
    const uint8_t* src = map.data;
    uint8_t* dst = bgr_frame.data;

    // ===== COLOR CONVERSION: BGRx (4-byte) → BGR (3-byte) =====
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // ===== SKIP PADDING BYTE IN BGRx FORMAT =====
            int src_idx = (y * width + x) * 4;
            int dst_idx = (y * width + x) * 3;
            dst[dst_idx + 0] = src[src_idx + 0];      // B
            dst[dst_idx + 1] = src[src_idx + 1];      // G
            dst[dst_idx + 2] = src[src_idx + 2];      // R
        }
    }

    gst_buffer_unmap(buffer, &map);

    // ===== PUBLISH FRAME =====
    owner_->publishDecodedFrame(bgr_frame);

    first_frame_received_.store(true);
    return true;
}

/**
 * @brief Main frame retrieval loop
 * 
 * Runs in dedicated thread, waits for decoded frames from GStreamer pipeline,
 * processes them (color conversion, GPU if enabled), and publishes to ROS.
 */
void GStreamerDecodedStreamDelegate::frameRetrievalLoop()
{
    while (running_.load()) {
        GstSample* sample = nullptr;

        // ===== WAIT FOR FRAME FROM QUEUE =====
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            // ===== TIMEOUT EVERY 100ms TO CHECK RUNNING FLAG =====
            const bool has_frame = frame_queue_cv_.wait_for(lock,
                std::chrono::milliseconds(100),
                [this] { return !frame_queue_.empty(); });

            if (!has_frame || frame_queue_.empty()) {
                continue;
            }

            sample = frame_queue_.front();
            frame_queue_.pop();
        }

        // ===== PROCESS FRAME OUTSIDE LOCK =====
        if (sample) {
            processCapturedFrame(sample);
            gst_sample_unref(sample);
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

/**
 * @brief Main function - ROS 2 node entry point
 */
int main(int argc, char* argv[])
{
    // ===== INITIALIZE GSTREAMER =====
    gst_init(&argc, &argv);

    // ===== INITIALIZE ROS 2 =====
    rclcpp::init(argc, argv);

    try {
        // ===== CREATE NODE INSTANCE =====
        auto node = std::make_shared<GStreamerDecodedNode>();

        // ===== START CAMERA AND STREAMING =====
        if (!node->startCamera()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to start camera");
            rclcpp::shutdown();
            return -1;
        }

        // ===== SPIN NODE (RUN) =====
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_gstreamer_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    // ===== SHUTDOWN ROS 2 =====
    rclcpp::shutdown();
    gst_deinit();
    return 0;
}
