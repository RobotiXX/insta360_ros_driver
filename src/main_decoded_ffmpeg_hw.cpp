#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class HardwareOnlyStreamDelegate : public ins_camera::StreamDelegate {
public:
    HardwareOnlyStreamDelegate(
        rclcpp::Node* node,
        int skip_frame,
        bool i_frame_only,
        const std::string& decoder_name)
        : node_(node),
          skip_frame_(skip_frame),
          i_frame_only_(i_frame_only),
          decoder_name_(decoder_name)
    {
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            "/dual_fisheye/image",
            rclcpp::SensorDataQoS());

        init_decoder();

        if (decoder_ready_) {
            RCLCPP_INFO(
                node_->get_logger(),
                "Hardware-only FFmpeg decoder ready (decoder=%s, skip_frame=%d, i_frame_only=%s)",
                decoder_name_.c_str(),
                skip_frame_,
                i_frame_only_ ? "true" : "false");
        }
    }

    ~HardwareOnlyStreamDelegate()
    {
        cleanup_decoder();
    }

    bool is_decoder_ready() const
    {
        return decoder_ready_;
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

        if (!seen_sps_) {
            seen_sps_ = hasNalType(data, size, 7);
        }
        if (!seen_pps_) {
            seen_pps_ = hasNalType(data, size, 8);
        }

        const bool has_idr = hasNalType(data, size, 5);
        if (!synced_on_idr_) {
            if (!(seen_sps_ && seen_pps_ && has_idr)) {
                return;
            }
            synced_on_idr_ = true;
            RCLCPP_INFO(node_->get_logger(), "SPS/PPS/IDR sync acquired for FFmpeg hardware decode feed.");
        }

        const uint8_t* cur = data;
        size_t remaining = size;
        while (remaining > 0) {
            int parsed = av_parser_parse2(
                parser_ctx_,
                codec_ctx_,
                &pkt_->data,
                &pkt_->size,
                cur,
                static_cast<int>(remaining),
                AV_NOPTS_VALUE,
                AV_NOPTS_VALUE,
                0);

            if (parsed < 0) {
                break;
            }

            cur += parsed;
            remaining -= static_cast<size_t>(parsed);

            if (pkt_->size <= 0) {
                continue;
            }

            if (i_frame_only_ && parser_ctx_->key_frame != 1) {
                av_packet_unref(pkt_);
                continue;
            }

            decode_and_publish_packet(pkt_);
            av_packet_unref(pkt_);
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override
    {
        (void)data;
    }

    void OnExposureData(const ins_camera::ExposureData& data) override
    {
        (void)data;
    }

private:
    bool hasNalType(const uint8_t* data, size_t size, uint8_t wanted_type) const
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

            if ((data[nal_idx] & 0x1f) == wanted_type) {
                return true;
            }
        }
        return false;
    }

    void init_decoder()
    {
        // Jetson hardware decode path only. Do not fallback to software.
        codec_ = avcodec_find_decoder_by_name(decoder_name_.c_str());
        if (!codec_) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Required hardware decoder '%s' is not available in ffmpeg build. Aborting.",
                decoder_name_.c_str());
            cleanup_decoder();
            return;
        }

        codec_ctx_ = avcodec_alloc_context3(codec_);
        if (!codec_ctx_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate codec context");
            cleanup_decoder();
            return;
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to open required hardware decoder '%s'. Aborting.",
                decoder_name_.c_str());
            cleanup_decoder();
            return;
        }

        parser_ctx_ = av_parser_init(AV_CODEC_ID_H264);
        if (!parser_ctx_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize H.264 parser");
            cleanup_decoder();
            return;
        }

        pkt_ = av_packet_alloc();
        frame_ = av_frame_alloc();
        if (!pkt_ || !frame_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate AV packet/frame");
            cleanup_decoder();
            return;
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
            ret = avcodec_receive_frame(codec_ctx_, frame_);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                return;
            }
            if (ret < 0) {
                return;
            }

            if (skip_frame_ > 0 && !i_frame_only_) {
                const bool should_publish = (decoded_frame_counter_++ % (skip_frame_ + 1) == 0);
                if (!should_publish) {
                    av_frame_unref(frame_);
                    continue;
                }
            }

            bool converted = false;

            if (frame_->format == AV_PIX_FMT_NV12 && frame_->data[0] && frame_->data[1]) {
                const int width = frame_->width;
                const int height = frame_->height;

                cv::Mat y_plane(height, width, CV_8UC1, frame_->data[0], frame_->linesize[0]);
                cv::Mat uv_plane(height / 2, width / 2, CV_8UC2, frame_->data[1], frame_->linesize[1]);
                cv::cvtColorTwoPlane(y_plane, uv_plane, bgr_frame_, cv::COLOR_YUV2BGR_NV12);
                converted = !bgr_frame_.empty();
            }

            if (!converted) {
                if (!sws_ctx_ && frame_->width > 0 && frame_->height > 0) {
                    sws_ctx_ = sws_getContext(
                        frame_->width,
                        frame_->height,
                        static_cast<AVPixelFormat>(frame_->format),
                        frame_->width,
                        frame_->height,
                        AV_PIX_FMT_BGR24,
                        SWS_FAST_BILINEAR,
                        nullptr,
                        nullptr,
                        nullptr);

                    if (!sws_ctx_) {
                        av_frame_unref(frame_);
                        return;
                    }

                    bgr_frame_.create(frame_->height, frame_->width, CV_8UC3);
                }

                if (sws_ctx_ && !bgr_frame_.empty()) {
                    uint8_t* dst_data[4] = {bgr_frame_.data, nullptr, nullptr, nullptr};
                    int dst_linesize[4] = {static_cast<int>(bgr_frame_.step[0]), 0, 0, 0};

                    sws_scale(
                        sws_ctx_,
                        (const uint8_t* const*)frame_->data,
                        frame_->linesize,
                        0,
                        frame_->height,
                        dst_data,
                        dst_linesize);
                    converted = true;
                }
            }

            if (converted && !bgr_frame_.empty()) {
                auto t0 = std::chrono::steady_clock::now();
                std_msgs::msg::Header header;
                header.stamp = node_->get_clock()->now();
                header.frame_id = "camera_frame";

                cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, bgr_frame_);
                auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
                cv_image.toImageMsg(*img_msg);
                image_pub_->publish(std::move(img_msg));
                auto t1 = std::chrono::steady_clock::now();

                publish_time_us_acc_ += static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
                ++published_frame_counter_;
            }

            ++decoded_frame_total_counter_;
            maybe_log_performance();

            av_frame_unref(frame_);
        }
    }

    void maybe_log_performance()
    {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - perf_window_start_).count();
        if (elapsed_ms < 2000) {
            return;
        }

        const double sec = static_cast<double>(elapsed_ms) / 1000.0;
        const double decoded_fps =
            sec > 0.0 ? static_cast<double>(decoded_frame_total_counter_) / sec : 0.0;
        const double published_fps =
            sec > 0.0 ? static_cast<double>(published_frame_counter_) / sec : 0.0;
        const double avg_publish_ms = published_frame_counter_ > 0
            ? static_cast<double>(publish_time_us_acc_) /
                  static_cast<double>(published_frame_counter_) /
                  1000.0
            : 0.0;

        RCLCPP_INFO(
            node_->get_logger(),
            "HW decode stats: decoded=%.1f fps, published=%.1f fps, avg_publish=%.2f ms, decoder=%s",
            decoded_fps,
            published_fps,
            avg_publish_ms,
            decoder_name_.c_str());

        decoded_frame_total_counter_ = 0;
        published_frame_counter_ = 0;
        publish_time_us_acc_ = 0;
        perf_window_start_ = now;
    }

    void cleanup_decoder()
    {
        decoder_ready_ = false;

        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }
        if (frame_) {
            av_frame_free(&frame_);
            frame_ = nullptr;
        }
        if (pkt_) {
            av_packet_free(&pkt_);
            pkt_ = nullptr;
        }
        if (parser_ctx_) {
            av_parser_close(parser_ctx_);
            parser_ctx_ = nullptr;
        }
        if (codec_ctx_) {
            avcodec_close(codec_ctx_);
            avcodec_free_context(&codec_ctx_);
            codec_ctx_ = nullptr;
        }
        codec_ = nullptr;
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVCodecParserContext* parser_ctx_ = nullptr;
    AVPacket* pkt_ = nullptr;
    AVFrame* frame_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;

    cv::Mat bgr_frame_;

    int skip_frame_ = 0;
    uint64_t decoded_frame_counter_ = 0;
    bool i_frame_only_ = false;
    bool decoder_ready_ = false;
    std::string decoder_name_;
    bool seen_sps_ = false;
    bool seen_pps_ = false;
    bool synced_on_idr_ = false;

    uint64_t decoded_frame_total_counter_ = 0;
    uint64_t published_frame_counter_ = 0;
    uint64_t publish_time_us_acc_ = 0;
    std::chrono::steady_clock::time_point perf_window_start_ = std::chrono::steady_clock::now();

    std::mutex decoder_mutex_;
};

class CameraDecodedHwNode : public rclcpp::Node {
public:
    CameraDecodedHwNode() : Node("insta360_decoded_ffmpeg_hw_node")
    {
        declare_parameter("skip_frame", 0);
        declare_parameter("i_frame_only", false);
        declare_parameter("enable_in_camera_stitching", false);
        declare_parameter("hw_decoder_name", "h264_nvv4l2dec");

        const int skip_frame = get_parameter("skip_frame").as_int();
        const bool i_frame_only = get_parameter("i_frame_only").as_bool();
        const bool enable_in_camera_stitching = get_parameter("enable_in_camera_stitching").as_bool();
        const std::string hw_decoder_name = get_parameter("hw_decoder_name").as_string();

        if (start_camera(skip_frame, i_frame_only, enable_in_camera_stitching, hw_decoder_name) != 0) {
            throw std::runtime_error("Failed to start camera stream with hardware-only decoder");
        }
    }

    ~CameraDecodedHwNode() override
    {
        if (cam_) {
            if (live_streaming_started_) {
                if (!cam_->StopLiveStreaming()) {
                    RCLCPP_WARN(get_logger(), "StopLiveStreaming failed during shutdown.");
                }
                live_streaming_started_ = false;
            }
            cam_->Close();
        }
    }

private:
    int start_camera(
        int skip_frame,
        bool i_frame_only,
        bool enable_in_camera_stitching,
        const std::string& hw_decoder_name)
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

        auto hw_delegate = std::make_shared<HardwareOnlyStreamDelegate>(
            this,
            skip_frame,
            i_frame_only,
            hw_decoder_name);
        if (!hw_delegate->is_decoder_ready()) {
            RCLCPP_ERROR(
                get_logger(),
                "Required hardware decoder initialization failed. Software fallback is disabled.");
            return -1;
        }

        std::shared_ptr<ins_camera::StreamDelegate> delegate = hw_delegate;
        stream_delegate_ = delegate;
        cam_->SetStreamDelegate(stream_delegate_);

        if (!cam_->EnableInCameraStitching(enable_in_camera_stitching)) {
            RCLCPP_WARN(
                get_logger(),
                "EnableInCameraStitching(%s) failed; continuing.",
                enable_in_camera_stitching ? "true" : "false");
        } else {
            RCLCPP_INFO(
                get_logger(),
                "EnableInCameraStitching(%s) applied.",
                enable_in_camera_stitching ? "true" : "false");
        }

        uint64_t utc_time = static_cast<uint64_t>(time(nullptr));
        cam_->SyncLocalTimeToCamera(utc_time);

        ins_camera::LiveStreamParam param;
        param.video_resolution = ins_camera::VideoResolution::RES_2560_1280P60;
        param.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
        param.video_bitrate = 1024 * 1024 * 2;
        param.enable_audio = false;
        param.using_lrv = false;

        if (!cam_->StartLiveStreaming(param)) {
            RCLCPP_ERROR(get_logger(), "Failed to start live streaming.");
            return -1;
        }

        live_streaming_started_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Live streaming started with strict hardware-only FFmpeg decoder (%s).",
            hw_decoder_name.c_str());
        return 0;
    }

    std::shared_ptr<ins_camera::Camera> cam_;
    std::shared_ptr<ins_camera::StreamDelegate> stream_delegate_;
    bool live_streaming_started_ = false;
};

int main(int argc, char* argv[])
{
    std::signal(SIGTSTP, [](int) {
        std::raise(SIGINT);
    });

    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<CameraDecodedHwNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        auto logger = rclcpp::get_logger("insta360_decoded_ffmpeg_hw_node");
        RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
