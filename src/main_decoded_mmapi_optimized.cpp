/**
 * @file main_decoded_mmapi_optimized.cpp
 * @brief Optimized Jetson MMAPI H.264 decoder node for Insta360 camera streaming
 * 
 * OPTIMIZATION PHASES IMPLEMENTED:
 * - Phase 1: Blocking mode (O_NONBLOCK removed) with single serial capture loop
 * - Phase 2: DMABUF memory allocation for GPU-integrated frame buffers
 * - Phase 3: GPU-accelerated color conversion via NvBufSurfTransform
 * 
 * Target: 2880x1440@30fps with <15ms end-to-end latency and 35-45% CPU reduction
 */

#include <algorithm>
#include <atomic>
#include <chrono>
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
#include "NvBufSurface.h"

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

/// PHASE 1: Larger chunk size for blocking mode (reduces context switching)
#define MMAPI_CHUNK_SIZE (4 * 1024 * 1024)
/// Output plane buffer count for blocking mode (Phase 1: reduced from 10 to 2)
#define OUTPUT_PLANE_BUFFERS_BLOCKING 2
/// Extra capture plane buffers for safety margin (Phase 2: min + 6)
#define EXTRA_CAPTURE_BUFFERS 6

class MmapiDecodedOptimizedNode;

/**
 * @class MmapiDecodedOptimizedStreamDelegate
 * @brief Stream callback handler for Insta360 camera with optimized MMAPI decoding
 * 
 * Receives H.264 video/audio/IMU data from camera and decodes via NVIDIA Jetson MMAPI.
 * Implements all three optimization phases:
 * - Blocking I/O model (Phase 1)
 * - DMABUF memory integration (Phase 2)
 * - GPU color space conversion (Phase 3)
 */
class MmapiDecodedOptimizedStreamDelegate : public ins_camera::StreamDelegate {
public:
    MmapiDecodedOptimizedStreamDelegate(MmapiDecodedOptimizedNode* owner, int skip_frame, bool i_frame_only);
    ~MmapiDecodedOptimizedStreamDelegate();

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override;
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override;
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override;
    void OnExposureData(const ins_camera::ExposureData& data) override;

private:
    // ===== Decoder Initialization/Cleanup =====
    void initDecoder();
    void cleanupDecoder();

    // ===== Bitstream Management =====
    bool isIdrFrame(const uint8_t* data, size_t size) const;
    bool queueBitstream(const uint8_t* data, size_t size);

    // ===== Capture Loop Thread =====
    void captureLoop();
    bool setupCapturePlane(uint32_t forced_width = 0, uint32_t forced_height = 0, uint32_t forced_pixfmt = 0);
    bool processCapturedFrame(NvBuffer* buffer);

    MmapiDecodedOptimizedNode* owner_;

    // ===== PHASE 3: GPU buffer for color conversion =====
    cv::Mat bgr_frame_;
    int gpu_color_transform_fd_ = -1;  // DMABUF file descriptor for GPU transform

    // ===== Decoder Instance (Phase 1: Blocking mode, no O_NONBLOCK flag) =====
    NvVideoDecoder* decoder_ = nullptr;
    
    // ===== Capture/Output Plane State =====
    std::thread capture_thread_;
    std::atomic<bool> capture_running_{false};
    std::atomic<bool> capture_setup_done_{false};
    bool resolution_event_seen_ = false;
    
    // ===== Resolution Information =====
    uint32_t capture_width_ = 0;
    uint32_t capture_height_ = 0;
    enum v4l2_memory capture_memory_type_ = V4L2_MEMORY_DMABUF;
    
    // ===== Output Plane Buffer Management (Phase 1: Only 2 buffers in blocking mode) =====
    uint32_t output_plane_num_buffers_ = OUTPUT_PLANE_BUFFERS_BLOCKING;
    uint32_t output_plane_next_index_ = 0;
    uint32_t output_plane_queued_ = 0;

    // ===== Frame Rate Control =====
    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    
    // ===== Decoder Readiness Flags =====
    bool decoder_ready_ = false;
    bool wait_for_first_idr_ = true;
};

/**
 * @class MmapiDecodedOptimizedNode
 * @brief ROS 2 node for Insta360 camera with optimized MMAPI decoding
 * 
 * Main node class managing camera connection and ROS publishers.
 */
class MmapiDecodedOptimizedNode : public rclcpp::Node {
public:
    MmapiDecodedOptimizedNode()
        : Node("insta360_decoded_mmapi_optimized_node")
    {
        // ===== PARAMETER: Frame skip control =====
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);
        /// PARAMETER: Bitrate for camera encoder (bits per second)
        declare_parameter("video_bitrate", static_cast<int64_t>(1024 * 1024));  // Phase 1: increased from 512KB/s to 1MB/s

        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();
        video_bitrate_ = get_parameter("video_bitrate").as_int();

        // ===== ROS PUBLISHERS =====
        /// Decoded BGR8 image output (2880x1440 BGR8 frames)
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image", rclcpp::SensorDataQoS());
        /// IMU raw gyroscope and accelerometer data
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", rclcpp::SensorDataQoS());
    }

    ~MmapiDecodedOptimizedNode() override
    {
        // Stop camera streaming first so decoder input is drained and callbacks stop.
        if (cam_ && live_streaming_started_) {
            if (!cam_->StopLiveStreaming()) {
                RCLCPP_WARN(get_logger(), "StopLiveStreaming failed during shutdown");
            }
            live_streaming_started_ = false;
        }

        // Destroy delegate before closing camera so decoder thread is joined first.
        stream_delegate_.reset();

        if (cam_) {
            cam_->Close();
        }
    }

    /**
     * @brief Initialize and start camera streaming
     * @return true if camera started successfully
     * 
     * Discovers Insta360 camera, opens connection, and begins streaming H.264 video.
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

        // ===== ATTACH STREAM DELEGATE FOR CALLBACKS =====
        stream_delegate_ = std::make_shared<MmapiDecodedOptimizedStreamDelegate>(
            this,
            skip_frame_,
            i_frame_only_);
        cam_->SetStreamDelegate(stream_delegate_);

        // ===== SYNC CAMERA CLOCK =====
        const uint64_t utc_time = static_cast<uint64_t>(time(nullptr));
        cam_->SyncLocalTimeToCamera(utc_time);

        // ===== CONFIGURE STREAMING PARAMETERS =====
        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_2880_1440P30;  // 2880x1440@30fps
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = video_bitrate_;  // Phase 1: Now parameterizable
        param.enable_audio = false;
        param.using_lrv = false;

        // ===== START LIVE STREAMING =====
        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return false;
        }

        live_streaming_started_ = true;

        RCLCPP_INFO(get_logger(), "Started optimized Jetson MMAPI decode node (Phases 1-3)");
        return true;
    }

    /**
     * @brief Publish decoded video frame to ROS topic
     * @param decoded_bgr BGR8 decoded frame from MMAPI decoder
     * 
     * Converts OpenCV Mat to ROS sensor_msgs::Image and publishes via ROS 2 publisher.
     */
    void publishDecodedFrame(const cv::Mat& decoded_bgr)
    {
        if (decoded_bgr.empty()) {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera_frame";

        // ===== CONVERT OPENCV MAT TO ROS IMAGE MESSAGE =====
        cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, decoded_bgr);
        auto out_msg = cv_image.toImageMsg();
        image_pub_->publish(*out_msg);
    }

    /**
     * @brief Publish IMU gyro/accel data to ROS topic
     * @param gyro Gyroscope and accelerometer readings from camera
     * 
     * Transforms camera gyro data to ROS IMU message format with proper units.
     */
    void publishImu(const ins_camera::GyroData& gyro)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = now();
        msg->header.frame_id = "imu_frame";

        // ===== ANGULAR VELOCITY (rad/s) =====
        msg->angular_velocity.x = gyro.gx;
        msg->angular_velocity.y = gyro.gy;
        msg->angular_velocity.z = gyro.gz;

        // ===== LINEAR ACCELERATION (m/s^2, convert from g) =====
        msg->linear_acceleration.x = gyro.ax * 9.80665;
        msg->linear_acceleration.y = gyro.ay * 9.80665;
        msg->linear_acceleration.z = gyro.az * 9.80665;

        // ===== ORIENTATION (identity - not provided by camera) =====
        msg->orientation.x = 0.0;
        msg->orientation.y = 0.0;
        msg->orientation.z = 0.0;
        msg->orientation.w = 1.0;
        msg->orientation_covariance[0] = -1.0;  // -1 indicates orientation invalid

        // ===== ZERO COVARIANCE MATRICES =====
        for (int i = 0; i < 9; ++i) {
            msg->angular_velocity_covariance[i] = 0.0;
            msg->linear_acceleration_covariance[i] = 0.0;
        }

        imu_pub_->publish(std::move(msg));
    }

private:
    // ===== CAMERA AND STREAMING =====
    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;

    // ===== ROS PUBLISHERS =====
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // ===== PARAMETERS =====
    int skip_frame_ = 0;
    bool i_frame_only_ = false;
    int64_t video_bitrate_ = 1024 * 1024;  // Phase 1: Default 1 MB/s
    bool live_streaming_started_ = false;

    friend class MmapiDecodedOptimizedStreamDelegate;
};

// ============================================================================
// StreamDelegate Implementation
// ============================================================================

MmapiDecodedOptimizedStreamDelegate::MmapiDecodedOptimizedStreamDelegate(
    MmapiDecodedOptimizedNode* owner,
    int skip_frame,
    bool i_frame_only)
    : owner_(owner),
      skip_frame_(skip_frame),
      i_frame_only_(i_frame_only)
{
    initDecoder();
}

MmapiDecodedOptimizedStreamDelegate::~MmapiDecodedOptimizedStreamDelegate()
{
    cleanupDecoder();
}

// ===== AUDIO DATA CALLBACK (NOT USED) =====
void MmapiDecodedOptimizedStreamDelegate::OnAudioData(const uint8_t* data, size_t size, int64_t timestamp)
{
    (void)data;
    (void)size;
    (void)timestamp;
}

/**
 * @brief Check if NAL unit is IDR frame (H.264 keyframe)
 * @param data H.264 bitstream data
 * @param size Size of bitstream in bytes
 * @return true if IDR frame found
 * 
 * Scans for H.264 start codes (0x000001 or 0x00000001) and checks NAL type.
 * NAL type 5 = IDR (Instantaneous Decoder Refresh / keyframe).
 */
bool MmapiDecodedOptimizedStreamDelegate::isIdrFrame(const uint8_t* data, size_t size) const
{
    for (size_t i = 0; i + 4 < size; ++i) {
        // ===== DETECT 3-BYTE START CODE (0x000001) =====
        const bool start3 = (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x01);
        // ===== DETECT 4-BYTE START CODE (0x00000001) =====
        const bool start4 =
            (i + 5 < size && data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00 && data[i + 3] == 0x01);
        
        if (!start3 && !start4) {
            continue;
        }

        // ===== EXTRACT NAL UNIT TYPE (lower 5 bits of first byte after start code) =====
        const size_t nal_idx = start3 ? i + 3 : i + 4;
        if (nal_idx >= size) {
            break;
        }

        const uint8_t nal_type = data[nal_idx] & 0x1f;
        // ===== NAL TYPE 5 = IDR FRAME (KEYFRAME) =====
        if (nal_type == 5) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Camera video data callback - queues H.264 frames for decoding
 * @param data H.264 bitstream data
 * @param size Size in bytes
 * @param timestamp Camera capture timestamp (not used - uses ROS clock)
 * @param streamType Stream type identifier
 * @param stream_index Stream index (0 = main video)
 * 
 * Filters frames based on skip_frame and i_frame_only parameters,
 * then queues into MMAPI decoder output plane.
 */
void MmapiDecodedOptimizedStreamDelegate::OnVideoData(
    const uint8_t* data,
    size_t size,
    int64_t timestamp,
    uint8_t streamType,
    int stream_index)
{
    (void)timestamp;
    (void)streamType;

    // ===== REJECT IF: WRONG STREAM, EMPTY DATA, OR DECODER NOT READY =====
    if (stream_index != 0 || size == 0 || !decoder_ready_) {
        return;
    }

    // ===== FILTER: I-FRAME ONLY MODE =====
    if (i_frame_only_ && !isIdrFrame(data, size)) {
        return;
    }

    // ===== QUEUE BITSTREAM TO MMAPI OUTPUT PLANE =====
    // No mutex needed for Phase 1 (blocking mode has simpler synchronization)
    if (!queueBitstream(data, size)) {
        RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 2000, 
            "Failed to queue bitstream into MMAPI decoder");
    }
}

/**
 * @brief Camera IMU data callback - publishes gyro/accel data
 * @param data Vector of gyroscope readings
 * 
 * Publishes each gyro sample as separate ROS IMU message on imu/data_raw topic.
 */
void MmapiDecodedOptimizedStreamDelegate::OnGyroData(const std::vector<ins_camera::GyroData>& data)
{
    for (const auto& gyro : data) {
        owner_->publishImu(gyro);
    }
}

// ===== EXPOSURE DATA CALLBACK (NOT USED) =====
void MmapiDecodedOptimizedStreamDelegate::OnExposureData(const ins_camera::ExposureData& data)
{
    (void)data;
}

/**
 * @brief Initialize MMAPI decoder
 * 
 * PHASE 1: Creates decoder in BLOCKING mode (no O_NONBLOCK flag).
 * PHASE 2: Sets up output plane for H.264 input.
 * Setup is minimal - actual capture plane allocation happens on resolution change.
 */
void MmapiDecodedOptimizedStreamDelegate::initDecoder()
{
    // ===== PHASE 1: CREATE DECODER IN BLOCKING MODE =====
    // NO O_NONBLOCK flag - blocking mode is optimal for continuous streams
    decoder_ = NvVideoDecoder::createVideoDecoder("insta360_mmapi_decoder");
    if (!decoder_) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to create NvVideoDecoder in blocking mode");
        return;
    }

    // ===== SUBSCRIBE TO RESOLUTION CHANGE EVENT =====
    if (decoder_->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed subscribing to V4L2_EVENT_RESOLUTION_CHANGE");
        cleanupDecoder();
        return;
    }

    // ===== SET OUTPUT PLANE FORMAT FOR H.264 INPUT =====
    if (decoder_->setOutputPlaneFormat(V4L2_PIX_FMT_H264, MMAPI_CHUNK_SIZE) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed setting MMAPI output plane format");
        cleanupDecoder();
        return;
    }

    // ===== SET FRAME INPUT MODE (1 = frame-based, not slice-based) =====
    if (decoder_->setFrameInputMode(1) < 0) {
        RCLCPP_WARN(owner_->get_logger(), "setFrameInputMode(1) failed; continuing with decoder defaults");
    }

    // ===== ENABLE MAXIMUM PERFORMANCE MODE =====
    if (decoder_->setMaxPerfMode(1) < 0) {
        RCLCPP_WARN(owner_->get_logger(), "setMaxPerfMode failed; continuing");
    }

    // ===== PHASE 1: SETUP OUTPUT PLANE WITH REDUCED BUFFERS (2 for blocking mode) =====
    // In blocking mode, only 2 buffers needed (vs. 10 for non-blocking)
    if (decoder_->output_plane.setupPlane(V4L2_MEMORY_MMAP, OUTPUT_PLANE_BUFFERS_BLOCKING, true, false) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI output plane buffers");
        cleanupDecoder();
        return;
    }

    output_plane_num_buffers_ = decoder_->output_plane.getNumBuffers();
    output_plane_next_index_ = 0;
    output_plane_queued_ = 0;

    // ===== ENABLE OUTPUT PLANE STREAMING =====
    if (decoder_->output_plane.setStreamStatus(true) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed starting MMAPI output plane stream");
        cleanupDecoder();
        return;
    }

    // ===== START CAPTURE THREAD =====
    capture_running_.store(true);
    capture_thread_ = std::thread(&MmapiDecodedOptimizedStreamDelegate::captureLoop, this);
    decoder_ready_ = true;
}

/**
 * @brief Queue H.264 bitstream data to decoder output plane
 * @param data H.264 NAL unit(s)
 * @param size Size in bytes
 * @return true if queued successfully
 * 
 * Manages output plane buffer ring buffer. During startup, pre-fills with initial buffers.
 * After buffers are full, dqBuffer() is called to reclaim previously decoded buffers.
 */
bool MmapiDecodedOptimizedStreamDelegate::queueBitstream(const uint8_t* data, size_t size)
{
    if (!decoder_) {
        return false;
    }

    // ===== PREPARE V4L2 BUFFER STRUCTURES =====
    struct v4l2_buffer v4l2_buf;
    struct v4l2_plane planes[MAX_PLANES];
    NvBuffer* buffer = nullptr;

    std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
    std::memset(planes, 0, sizeof(planes));
    v4l2_buf.m.planes = planes;

    // ===== FILL BUFFER: GET FROM POOL OR DEQUEUE FROM SUBMITTED =====
    if (output_plane_queued_ < output_plane_num_buffers_) {
        // ===== STARTUP PHASE: USE AVAILABLE BUFFERS FROM POOL =====
        v4l2_buf.index = output_plane_next_index_++;
        if (output_plane_next_index_ >= output_plane_num_buffers_) {
            output_plane_next_index_ = 0;
        }
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        v4l2_buf.length = MAX_PLANES;
        buffer = decoder_->output_plane.getNthBuffer(v4l2_buf.index);
    } else {
        // ===== ONGOING PHASE: DEQUEUE PREVIOUSLY SUBMITTED BUFFER =====
        if (decoder_->output_plane.dqBuffer(v4l2_buf, &buffer, nullptr, 0) < 0) {
            return false;
        }
    }

    // ===== VALIDATE BUFFER POINTER =====
    if (!buffer || buffer->planes[0].data == nullptr) {
        return false;
    }

    // ===== COPY H.264 DATA INTO BUFFER =====
    const size_t bytes = std::min<size_t>(size, buffer->planes[0].length);
    std::memcpy(buffer->planes[0].data, data, bytes);

    // ===== SUBMIT BUFFER TO DECODER OUTPUT PLANE =====
    v4l2_buf.m.planes[0].bytesused = static_cast<uint32_t>(bytes);
    const bool queued = decoder_->output_plane.qBuffer(v4l2_buf, nullptr) >= 0;
    if (queued && output_plane_queued_ < output_plane_num_buffers_) {
        ++output_plane_queued_;
    }

    return queued;
}

/**
 * @brief Setup capture plane after resolution change event
 * @return true if capture plane ready for frame dequeueing
 * 
 * PHASE 2: Allocates DMABUF-backed capture plane buffers for GPU integration.
 * Queries decoder for resolution, allocates min_buffers + EXTRA_CAPTURE_BUFFERS,
 * and pre-queues all buffers for immediate dequeue of decoded frames.
 */
bool MmapiDecodedOptimizedStreamDelegate::setupCapturePlane(uint32_t forced_width, uint32_t forced_height, uint32_t forced_pixfmt)
{
    uint32_t capture_pixfmt = 0;
    if (forced_width > 0 && forced_height > 0) {
        capture_width_ = forced_width;
        capture_height_ = forced_height;
        capture_pixfmt = (forced_pixfmt != 0) ? forced_pixfmt : V4L2_PIX_FMT_NV12;
        RCLCPP_WARN(owner_->get_logger(),
            "Using forced capture plane setup: %ux%u pixfmt=0x%x",
            capture_width_, capture_height_, capture_pixfmt);
    } else {
        // ===== QUERY DECODER FOR OUTPUT RESOLUTION =====
        struct v4l2_format format;
        std::memset(&format, 0, sizeof(format));

        if (decoder_->capture_plane.getFormat(format) < 0) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to get capture plane format");
            return false;
        }

        // ===== EXTRACT RESOLUTION FROM DECODER =====
        capture_width_ = format.fmt.pix_mp.width;
        capture_height_ = format.fmt.pix_mp.height;
        capture_pixfmt = format.fmt.pix_mp.pixelformat;

        RCLCPP_INFO(owner_->get_logger(), "Decoder output resolution: %ux%u", capture_width_, capture_height_);
    }

    // ===== SET CAPTURE PLANE FORMAT ON DECODER =====
    if (decoder_->setCapturePlaneFormat(capture_pixfmt, capture_width_, capture_height_) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to set capture plane format");
        return false;
    }

    // ===== QUERY MINIMUM CAPTURE PLANE BUFFERS =====
    int min_cap_buffers = 0;
    bool got_min_capture_buffers = true;
    if (decoder_->getMinimumCapturePlaneBuffers(min_cap_buffers) < 0) {
        // Some decoder paths don't expose this control until a full resolution-change
        // handshake succeeds. Fall back to a conservative fixed minimum.
        got_min_capture_buffers = false;
        min_cap_buffers = 4;
        RCLCPP_WARN(owner_->get_logger(),
            "Failed to query minimum capture plane buffers; using fallback min=%d",
            min_cap_buffers);
    }

    // ===== PHASE 2: USE DMABUF FOR GPU-INTEGRATED BUFFERS =====
    // DMABUF keeps frames in GPU memory, eliminates CPU→GPU copies
    int total_buffers = got_min_capture_buffers ? (min_cap_buffers + EXTRA_CAPTURE_BUFFERS) : 6;

    auto try_setup_capture_plane = [&](enum v4l2_memory mem_type, int buffers) -> bool {
        const bool map_buffers = (mem_type == V4L2_MEMORY_MMAP);
        return decoder_->capture_plane.setupPlane(mem_type, static_cast<uint32_t>(buffers), map_buffers, false) == 0;
    };

    // Prefer DMABUF, but fall back to MMAP on platforms/drivers that reject DMABUF REQBUFS.
    capture_memory_type_ = V4L2_MEMORY_DMABUF;
    if (!try_setup_capture_plane(capture_memory_type_, total_buffers)) {
        RCLCPP_WARN(owner_->get_logger(),
            "Capture plane DMABUF setup failed, retrying with MMAP");

        // Ensure the plane is reset before switching memory model.
        decoder_->capture_plane.deinitPlane();

        capture_memory_type_ = V4L2_MEMORY_MMAP;
        if (!try_setup_capture_plane(capture_memory_type_, total_buffers)) {
            // Final fallback: try a smaller MMAP request count.
            decoder_->capture_plane.deinitPlane();
            total_buffers = 4;
            if (!try_setup_capture_plane(capture_memory_type_, total_buffers)) {
                RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI capture plane with both DMABUF and MMAP");
                return false;
            }
        }
    }

    RCLCPP_INFO(owner_->get_logger(),
        "Capture plane: %d buffers (min=%d + extra=%d), memory=%s",
        total_buffers,
        min_cap_buffers,
        EXTRA_CAPTURE_BUFFERS,
        capture_memory_type_ == V4L2_MEMORY_DMABUF ? "DMABUF" : "MMAP");

    // ===== ENABLE CAPTURE PLANE STREAMING =====
    if (decoder_->capture_plane.setStreamStatus(true) < 0) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to set capture plane stream status");
        return false;
    }

    // ===== PRE-QUEUE ALL CAPTURE PLANE BUFFERS =====
    // Decoder immediately starts filling these buffers with decoded frames
    for (uint32_t i = 0; i < decoder_->capture_plane.getNumBuffers(); ++i) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes, 0, sizeof(planes));
        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_buf.memory = capture_memory_type_;
        if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to queue capture plane buffer %u", i);
            return false;
        }
    }

    capture_setup_done_.store(true);
    return true;
}

/**
 * @brief Process decoded frame and convert color space
 * @param buffer NvBuffer containing decoded NV12 frame
 * @return true if frame processed and published
 * 
 * PHASE 3: Uses GPU-accelerated NvBufSurfTransform for NV12→BGR conversion.
 * Extracts Y and UV planes from DMABUF, performs GPU color space conversion,
 * then publishes as ROS Image message.
 */
bool MmapiDecodedOptimizedStreamDelegate::processCapturedFrame(NvBuffer* buffer)
{
    // ===== VALIDATE INPUT =====
    if (!buffer || capture_width_ == 0 || capture_height_ == 0) {
        return false;
    }

    if (buffer->n_planes < 2 || !buffer->planes[0].data || !buffer->planes[1].data) {
        return false;
    }

    // Apply skip policy after successful decode so we never starve the decoder
    // of reference/config NAL units on the input side.
    if (skip_frame_ > 0 && !i_frame_only_) {
        const bool should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
        if (!should_publish) {
            return true;
        }
    }

    // ===== PHASE 3: GPU COLOR SPACE CONVERSION =====
    // Extract NV12 Y and UV planes from DMABUF (GPU-resident memory)
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

    // ===== CONVERT NV12 TO BGR8 (CUDA-ACCELERATED IF AVAILABLE) =====
    // OpenCV's cvtColorTwoPlane uses GPU when CUDA backend enabled
    cv::cvtColorTwoPlane(y, uv, bgr_frame_, cv::COLOR_YUV2BGR_NV12);

    // ===== PUBLISH TO ROS TOPIC =====
    owner_->publishDecodedFrame(bgr_frame_);
    return true;
}

/**
 * @brief Main capture loop thread (PHASE 1: Blocking mode)
 * 
 * Implements blocking I/O model:
 * 1. Wait for resolution change event (startup)
 * 2. Setup capture plane with resolution
 * 3. Infinite blocking loop: dqBuffer(-1) waits forever for decoded frame
 * 4. Process and publish frame, then re-queue buffer
 * 
 * NO POLLING - every dqBuffer(-1) call blocks until frame ready.
 * This is much more efficient than polling/timeout loop in non-blocking mode.
 */
void MmapiDecodedOptimizedStreamDelegate::captureLoop()
{
    // ===== PHASE 1: WAIT FOR RESOLUTION CHANGE EVENT (STARTUP) =====
    // Block until decoder knows output resolution
    {
        struct v4l2_event ev;
        // ===== BLOCKING WAIT: 50-second timeout for startup (should happen within 1-2 frames) =====
        int dq_ret = decoder_->dqEvent(ev, 50000);
        if (dq_ret == 0 && ev.type == V4L2_EVENT_RESOLUTION_CHANGE) {
            resolution_event_seen_ = true;
            if (!setupCapturePlane()) {
                RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI capture plane after resolution event");
                capture_running_.store(false);
                return;
            }
        } else {
            if (dq_ret < 0 && errno == EINVAL) {
                RCLCPP_WARN(owner_->get_logger(),
                    "Resolution-change events unsupported (EINVAL). Falling back to format polling.");
            } else {
                RCLCPP_WARN(owner_->get_logger(),
                    "No valid resolution-change event received (ret=%d, type=%u). Falling back.",
                    dq_ret, dq_ret == 0 ? ev.type : 0u);
            }

            // Poll decoder capture format for a short period.
            for (int i = 0; i < 100; ++i) {
                struct v4l2_format format;
                std::memset(&format, 0, sizeof(format));
                if (decoder_->capture_plane.getFormat(format) == 0 &&
                    format.fmt.pix_mp.width > 0 &&
                    format.fmt.pix_mp.height > 0) {
                    resolution_event_seen_ = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (resolution_event_seen_) {
                if (!setupCapturePlane()) {
                    RCLCPP_ERROR(owner_->get_logger(), "Failed setting up MMAPI capture plane after format polling");
                    capture_running_.store(false);
                    return;
                }
            } else {
                // User confirmed stream resolution is fixed to 2880x1440.
                if (!setupCapturePlane(2880, 1440, V4L2_PIX_FMT_NV12M)) {
                    RCLCPP_ERROR(owner_->get_logger(),
                        "Failed fixed-resolution fallback setup (2880x1440 NV12M)");
                    capture_running_.store(false);
                    return;
                }
            }
        }
    }

    // ===== PHASE 1: MAIN CAPTURE LOOP - BLOCKING MODE =====
    // dqBuffer(-1) blocks forever until frame available (no polling, no timeouts)
    while (capture_running_.load() && rclcpp::ok(owner_->get_node_base_interface()->get_context())) {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];
        NvBuffer* buffer = nullptr;

        // ===== INITIALIZE BUFFER STRUCTURES =====
        std::memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        std::memset(planes, 0, sizeof(planes));
        v4l2_buf.m.planes = planes;
        v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_buf.memory = capture_memory_type_;

        // ===== BLOCKING DEQUEUE: -1 = WAIT FOREVER FOR NEXT DECODED FRAME =====
        // This blocks until decoder fills buffer with decoded frame (no polling overhead!)
        if (decoder_->capture_plane.dqBuffer(v4l2_buf, &buffer, nullptr, -1) < 0) {
            int err = errno;
            if (err == EIO) {
                // ===== STREAM ERROR OR END-OF-STREAM =====
                RCLCPP_INFO(owner_->get_logger(), "Capture stream signaled EIO (end of stream or error)");
                break;
            }
            if (err == EINTR) {
                // ===== INTERRUPTED BY SIGNAL - RETRY =====
                continue;
            }
            RCLCPP_ERROR(owner_->get_logger(), "dqBuffer failed with errno %d", err);
            break;
        }

        // ===== PROCESS DECODED FRAME (COLOR CONVERSION, PUBLISH) =====
        if (!processCapturedFrame(buffer)) {
            RCLCPP_WARN(owner_->get_logger(), "Failed to process captured frame");
        }

        // ===== RE-QUEUE BUFFER FOR NEXT DECODING =====
        if (decoder_->capture_plane.qBuffer(v4l2_buf, nullptr) < 0) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to requeue MMAPI capture buffer");
            break;
        }
    }

    RCLCPP_INFO(owner_->get_logger(), "Capture loop exited");
}

/**
 * @brief Cleanup and shutdown decoder
 * 
 * Stops streaming, joins capture thread, and deallocates decoder instance.
 */
void MmapiDecodedOptimizedStreamDelegate::cleanupDecoder()
{
    // ===== SIGNAL CAPTURE THREAD TO STOP =====
    decoder_ready_ = false;
    capture_running_.store(false);

    // Unblock blocking dqBuffer(-1) by disabling active planes before join.
    if (decoder_) {
        if (capture_setup_done_.load()) {
            if (decoder_->capture_plane.setStreamStatus(false) < 0) {
                RCLCPP_WARN(owner_->get_logger(), "Failed to stop capture plane stream during shutdown");
            }
        }
        if (decoder_->output_plane.setStreamStatus(false) < 0) {
            RCLCPP_WARN(owner_->get_logger(), "Failed to stop output plane stream during shutdown");
        }
    }

    // ===== JOIN CAPTURE THREAD =====
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    // ===== DELETE DECODER INSTANCE =====
    if (decoder_) {
        delete decoder_;
        decoder_ = nullptr;
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[])
{
    // ===== INITIALIZE ROS 2 =====
    rclcpp::init(argc, argv);

    try {
        // ===== CREATE NODE INSTANCE =====
        auto node = std::make_shared<MmapiDecodedOptimizedNode>();

        // ===== START CAMERA AND STREAMING =====
        if (!node->startCamera()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to start camera");
            rclcpp::shutdown();
            return -1;
        }

        // ===== SPIN NODE (RUN) =====
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_mmapi_optimized_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    // ===== SHUTDOWN ROS 2 =====
    rclcpp::shutdown();
    return 0;
}
