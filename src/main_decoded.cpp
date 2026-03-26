#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

static enum AVPixelFormat get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts)
{
    (void)ctx;
    const enum AVPixelFormat *p;
    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_CUDA) {
            return *p;
        }
    }
    return AV_PIX_FMT_NONE;
}

class DecodingStreamDelegate : public ins_camera::StreamDelegate {
public:
    DecodingStreamDelegate(
        rclcpp::Node* node,
        int skip_frame,
        bool i_frame_only)
        : node_(node),
          skip_frame_(skip_frame),
          i_frame_only_(i_frame_only)
    {
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image",
            rclcpp::SensorDataQoS());
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            rclcpp::SensorDataQoS());

        init_decoder();
        RCLCPP_INFO(
            node_->get_logger(),
            "Decoded image + IMU publishers ready (skip_frame=%d, i_frame_only=%s)",
            skip_frame_,
            i_frame_only_ ? "true" : "false");
    }

    ~DecodingStreamDelegate()
    {
        cleanup_decoder();
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override
    {
        (void)data;
        (void)size;
        (void)timestamp;
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index) override
    {
        (void)timestamp;
        (void)streamType;

        if (stream_index != 0 || size == 0 || !decoder_ready_) {
            return;
        }

        std::lock_guard<std::mutex> lock(decoder_mutex_);

        const uint8_t* cur_data = data;
        size_t remaining_size = size;

        while (remaining_size > 0) {
            int bytes_parsed = av_parser_parse2(
                parser_ctx_,
                codec_ctx_,
                &pkt_->data,
                &pkt_->size,
                cur_data,
                static_cast<int>(remaining_size),
                AV_NOPTS_VALUE,
                AV_NOPTS_VALUE,
                0);

            if (bytes_parsed < 0) {
                break;
            }

            cur_data += bytes_parsed;
            remaining_size -= static_cast<size_t>(bytes_parsed);

            if (pkt_->size <= 0) {
                continue;
            }

            if (i_frame_only_ && parser_ctx_->key_frame != 1) {
                continue;
            }

            if (skip_frame_ > 0 && !i_frame_only_) {
                bool should_publish = (frame_counter_++ % (skip_frame_ + 1) == 0);
                if (!should_publish) {
                    continue;
                }
            }

            decode_and_publish_packet(pkt_);
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override
    {
        for (const auto& gyro : data) {
            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp = node_->get_clock()->now();
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

            for (int i = 0; i < 9; i++) {
                msg->angular_velocity_covariance[i] = 0.0;
                msg->linear_acceleration_covariance[i] = 0.0;
            }

            imu_pub_->publish(std::move(msg));
        }
    }

    void OnExposureData(const ins_camera::ExposureData& data) override
    {
        (void)data;
    }

private:
    void init_decoder()
    {
        hw_type_ = AV_HWDEVICE_TYPE_CUDA;
        const char* decoder_name = "h264_cuvid";

        codec_ = avcodec_find_decoder_by_name(decoder_name);
        if (!codec_) {
            RCLCPP_WARN(node_->get_logger(), "NVDEC not available, falling back to software H.264 decoder");
            hw_type_ = AV_HWDEVICE_TYPE_NONE;
            codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
            if (!codec_) {
                RCLCPP_ERROR(node_->get_logger(), "No H.264 decoder available");
                cleanup_decoder();
                return;
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "Using NVDEC hardware decoder (h264_cuvid)");
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            int err = av_hwdevice_ctx_create(&hw_device_ctx_, hw_type_, nullptr, nullptr, 0);
            if (err < 0) {
                RCLCPP_WARN(node_->get_logger(), "Failed to create CUDA hw context; using software decode");
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
                codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
                if (!codec_) {
                    RCLCPP_ERROR(node_->get_logger(), "No H.264 decoder available");
                    cleanup_decoder();
                    return;
                }
            }
        }

        parser_ctx_ = av_parser_init(codec_->id);
        if (!parser_ctx_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize H.264 parser");
            cleanup_decoder();
            return;
        }

        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate codec context");
            cleanup_decoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open H.264 decoder");
            cleanup_decoder();
            return;
        }

        pkt_ = av_packet_alloc();
        hw_frame_ = av_frame_alloc();

        if (!pkt_ || !hw_frame_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate AV packet/frame");
            cleanup_decoder();
            return;
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to allocate software AV frame");
                cleanup_decoder();
                return;
            }
        }

        decoder_ready_ = true;
    }

    void decode_and_publish_packet(AVPacket* packet)
    {
        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret < 0) {
            return;
        }

        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx_, hw_frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            }
            if (ret < 0) {
                return;
            }

            AVFrame* frame_to_publish = hw_frame_;
            if (hw_frame_->format == AV_PIX_FMT_CUDA || hw_frame_->format == AV_PIX_FMT_VAAPI) {
                if (av_hwframe_transfer_data(sw_frame_, hw_frame_, 0) < 0) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
                frame_to_publish = sw_frame_;
            }

            if (!sws_ctx_ && frame_to_publish->width > 0 && frame_to_publish->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_publish->width,
                    frame_to_publish->height,
                    static_cast<AVPixelFormat>(frame_to_publish->format),
                    frame_to_publish->width,
                    frame_to_publish->height,
                    AV_PIX_FMT_BGR24,
                    SWS_POINT,
                    nullptr,
                    nullptr,
                    nullptr);

                if (!sws_ctx_) {
                    av_frame_unref(hw_frame_);
                    if (frame_to_publish == sw_frame_) {
                        av_frame_unref(sw_frame_);
                    }
                    return;
                }

                bgr_frame_.create(frame_to_publish->height, frame_to_publish->width, CV_8UC3);
            }

            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t* dst_data[4] = {bgr_frame_.data, nullptr, nullptr, nullptr};
                int dst_linesize[4] = {static_cast<int>(bgr_frame_.step[0]), 0, 0, 0};

                sws_scale(
                    sws_ctx_,
                    (const uint8_t* const*)frame_to_publish->data,
                    frame_to_publish->linesize,
                    0,
                    frame_to_publish->height,
                    dst_data,
                    dst_linesize);

                std_msgs::msg::Header header;
                header.stamp = node_->get_clock()->now();
                header.frame_id = "camera_frame";

                cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, bgr_frame_);
                auto img_msg = cv_image.toImageMsg();
                image_pub_->publish(*img_msg);
            }

            av_frame_unref(hw_frame_);
            if (frame_to_publish == sw_frame_) {
                av_frame_unref(sw_frame_);
            }
        }
    }

    void cleanup_decoder()
    {
        decoder_ready_ = false;

        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (sw_frame_) {
            av_frame_free(&sw_frame_);
            sw_frame_ = nullptr;
        }
        if (hw_frame_) {
            av_frame_free(&hw_frame_);
            hw_frame_ = nullptr;
        }
        if (pkt_) {
            av_packet_free(&pkt_);
            pkt_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_close(codec_ctx_);
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        if (parser_ctx_) {
            av_parser_close(parser_ctx_);
            parser_ctx_ = nullptr;
        }
        if (hw_device_ctx_) {
            av_buffer_unref(&hw_device_ctx_);
            hw_device_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVCodecParserContext* parser_ctx_ = nullptr;
    AVPacket* pkt_ = nullptr;
    AVFrame* hw_frame_ = nullptr;
    AVFrame* sw_frame_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    AVBufferRef *hw_device_ctx_ = nullptr;
    enum AVHWDeviceType hw_type_ = AV_HWDEVICE_TYPE_NONE;

    cv::Mat bgr_frame_;

    int skip_frame_ = 0;
    int frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool decoder_ready_ = false;

    std::mutex decoder_mutex_;
};

class CameraDecodedNode : public rclcpp::Node {
public:
    CameraDecodedNode() : Node("insta360_decoded_node")
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);

        const int skip_frame = get_parameter("skip_frame").as_int();
        const bool i_frame_only = get_parameter("i_frame_only").as_bool();

        if (start_camera(skip_frame, i_frame_only) != 0) {
            throw std::runtime_error("Failed to start camera stream");
        }
    }

    ~CameraDecodedNode() override
    {
        if (cam_) {
            cam_->Close();
        }
    }

private:
    int start_camera(int skip_frame, bool i_frame_only)
    {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) {
            RCLCPP_ERROR(get_logger(), "No available camera devices found.");
            return -1;
        }

        cam_ = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!cam_->Open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera.");
            discovery.FreeDeviceDescriptors(list);
            return -1;
        }
        RCLCPP_INFO(get_logger(), "Camera opened successfully.");
        discovery.FreeDeviceDescriptors(list);

        std::shared_ptr<ins_camera::StreamDelegate> delegate =
            std::make_shared<DecodingStreamDelegate>(this, skip_frame, i_frame_only);
        stream_delegate_ = delegate;
        cam_->SetStreamDelegate(delegate);

        auto start = time(NULL);
        uint64_t utc_time = static_cast<uint64_t>(start);
        uint32_t offset_time = 0;
        cam_->SyncLocalTimeToCamera(utc_time, offset_time);

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_1920_960P30;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 / 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return -1;
        }

        RCLCPP_INFO(get_logger(), "Live streaming started with integrated decode.");
        return 0;
    }

    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<CameraDecodedNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
