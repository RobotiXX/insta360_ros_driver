#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

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

class EquirectangularCropH264Node : public rclcpp::Node {
public:
    EquirectangularCropH264Node()
        : Node("equirectangular_crop_h264_node")
    {
        declare_parameter("compressed_topic", "/dual_fisheye/image/compressed");
        declare_parameter("output_topic", "/equirectangular/image");
        declare_parameter("camera_info_topic", "/equirectangular/image/camera_info");
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
        declare_parameter("gpu", true);
        declare_parameter("out_width", 1920);
        declare_parameter("out_height", 960);
        declare_parameter("crop_out_height", 640);
        declare_parameter("enable_seam_blending", false);

        compressed_topic_ = get_parameter("compressed_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        camera_info_topic_ = get_parameter("camera_info_topic").as_string();
        skip_frame_ = get_parameter("skip_frame").as_int();
        i_frame_only_ = get_parameter("i_frame_only").as_bool();

        load_parameters();
        update_camera_parameters();
        init_decoder();

        compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            compressed_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&EquirectangularCropH264Node::compressed_callback, this, std::placeholders::_1));
        equirect_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::SensorDataQoS());
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "Compressed-input cropped equirect node ready");
        RCLCPP_INFO(get_logger(), "  Subscribing to: %s", compressed_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Publishing to: %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Publishing camera info to: %s", camera_info_topic_.c_str());
    }

    ~EquirectangularCropH264Node() override
    {
        cleanup_decoder();
    }

private:
    void load_parameters()
    {
        cx_offset_ = get_parameter("cx_offset").as_double();
        cy_offset_ = get_parameter("cy_offset").as_double();
        front_cx_offset_ = get_parameter("front_cx_offset").as_double();
        front_cy_offset_ = get_parameter("front_cy_offset").as_double();
        back_cx_offset_ = get_parameter("back_cx_offset").as_double();
        back_cy_offset_ = get_parameter("back_cy_offset").as_double();
        crop_size_ = get_parameter("crop_size").as_int();
        out_width_ = get_parameter("out_width").as_int();
        out_height_ = get_parameter("out_height").as_int();
        crop_out_height_ = get_parameter("crop_out_height").as_int();
        gpu_enabled_ = get_parameter("gpu").as_bool();
        enable_seam_blending_ = get_parameter("enable_seam_blending").as_bool();

        auto translation = get_parameter("translation").as_double_array();
        tx_ = translation[0];
        ty_ = translation[1];
        tz_ = translation[2];

        auto rotation_deg = get_parameter("rotation_deg").as_double_array();
        roll_ = rotation_deg[0] * M_PI / 180.0;
        pitch_ = rotation_deg[1] * M_PI / 180.0;
        yaw_ = rotation_deg[2] * M_PI / 180.0;

        if (crop_out_height_ <= 0) {
            crop_out_height_ = 1;
        }
        if (crop_out_height_ > out_height_) {
            crop_out_height_ = out_height_;
        }
    }

    void update_camera_parameters()
    {
        cv::Mat rx = (cv::Mat_<double>(3, 3) <<
            1.0, 0.0, 0.0,
            0.0, std::cos(roll_), -std::sin(roll_),
            0.0, std::sin(roll_), std::cos(roll_));
        cv::Mat ry = (cv::Mat_<double>(3, 3) <<
            std::cos(pitch_), 0.0, std::sin(pitch_),
            0.0, 1.0, 0.0,
            -std::sin(pitch_), 0.0, std::cos(pitch_));
        cv::Mat rz = (cv::Mat_<double>(3, 3) <<
            std::cos(yaw_), -std::sin(yaw_), 0.0,
            std::sin(yaw_), std::cos(yaw_), 0.0,
            0.0, 0.0, 1.0);

        back_to_front_rotation_ = rz * ry * rx;
        back_to_front_translation_ = cv::Vec3d(tx_, ty_, tz_);
        maps_initialized_ = false;
    }

    void init_mapping(int img_height, int img_width)
    {
        img_height_ = img_height;
        img_width_ = img_width;

        const double base_cx = img_width / 2.0 + cx_offset_;
        const double base_cy = img_height / 2.0 + cy_offset_;
        const double front_cx = base_cx + front_cx_offset_;
        const double front_cy = base_cy + front_cy_offset_;
        const double back_cx = base_cx + back_cx_offset_;
        const double back_cy = base_cy + back_cy_offset_;

        cv::Mat x_grid, y_grid;
        cv::Mat x_range = cv::Mat::zeros(1, out_width_, CV_32F);
        cv::Mat y_range = cv::Mat::zeros(crop_out_height_, 1, CV_32F);
        const float crop_y_offset = static_cast<float>(out_height_ - crop_out_height_) / 2.0f;

        for (int x = 0; x < out_width_; ++x) {
            x_range.at<float>(0, x) = static_cast<float>(x);
        }
        for (int y = 0; y < crop_out_height_; ++y) {
            y_range.at<float>(y, 0) = static_cast<float>(y) + crop_y_offset;
        }

        cv::repeat(x_range, crop_out_height_, 1, x_grid);
        cv::repeat(y_range, 1, out_width_, y_grid);

        cv::Mat longitude = (x_grid / static_cast<float>(out_width_)) * 2.0f * static_cast<float>(M_PI) - static_cast<float>(M_PI);
        cv::Mat latitude = (y_grid / static_cast<float>(out_height_)) * static_cast<float>(M_PI) - static_cast<float>(M_PI) / 2.0f;

        cv::Mat X(crop_out_height_, out_width_, CV_32F);
        cv::Mat Y(crop_out_height_, out_width_, CV_32F);
        cv::Mat Z(crop_out_height_, out_width_, CV_32F);

        for (int y = 0; y < crop_out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                const float lat = latitude.at<float>(y, x);
                const float lon = longitude.at<float>(y, x);
                const float cos_lat = std::cos(lat);
                X.at<float>(y, x) = cos_lat * std::sin(lon);
                Y.at<float>(y, x) = std::sin(lat);
                Z.at<float>(y, x) = cos_lat * std::cos(lon);
            }
        }

        front_mask_ = Z >= 0;
        back_mask_ = Z < 0;

        front_map_x_ = cv::Mat::zeros(crop_out_height_, out_width_, CV_32F);
        front_map_y_ = cv::Mat::zeros(crop_out_height_, out_width_, CV_32F);
        back_map_x_ = cv::Mat::zeros(crop_out_height_, out_width_, CV_32F);
        back_map_y_ = cv::Mat::zeros(crop_out_height_, out_width_, CV_32F);

        for (int y = 0; y < crop_out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                if (!front_mask_.at<uchar>(y, x)) {
                    continue;
                }
                const float x_val = X.at<float>(y, x);
                const float y_val = Y.at<float>(y, x);
                const float z_val = Z.at<float>(y, x);
                float r = std::sqrt(x_val * x_val + y_val * y_val);
                if (r < 1e-6f) {
                    r = 1e-6f;
                }
                const float theta = std::atan2(r, std::fabs(z_val));
                const float r_fisheye = 2.0f * theta / static_cast<float>(M_PI) * (img_width / 2.0f);
                front_map_x_.at<float>(y, x) = static_cast<float>(front_cx) + x_val / r * r_fisheye;
                front_map_y_.at<float>(y, x) = static_cast<float>(front_cy) + y_val / r * r_fisheye;
            }
        }

        for (int y = 0; y < crop_out_height_; ++y) {
            for (int x = 0; x < out_width_; ++x) {
                if (!back_mask_.at<uchar>(y, x)) {
                    continue;
                }
                cv::Vec3d point(X.at<float>(y, x), Y.at<float>(y, x), Z.at<float>(y, x));
                const cv::Mat transformed = back_to_front_rotation_ * cv::Mat(point) + cv::Mat(back_to_front_translation_);
                const float x_back = static_cast<float>(-transformed.at<double>(0));
                const float y_back = static_cast<float>(transformed.at<double>(1));
                const float z_back = static_cast<float>(transformed.at<double>(2));
                float r = std::sqrt(x_back * x_back + y_back * y_back);
                if (r < 1e-6f) {
                    r = 1e-6f;
                }
                const float theta = std::atan2(r, std::fabs(z_back));
                const float r_fisheye = 2.0f * theta / static_cast<float>(M_PI) * (img_width / 2.0f);
                back_map_x_.at<float>(y, x) = static_cast<float>(back_cx) + x_back / r * r_fisheye;
                back_map_y_.at<float>(y, x) = static_cast<float>(back_cy) + y_back / r * r_fisheye;
            }
        }

        cv::Mat front_mask_uint8;
        front_mask_.convertTo(front_mask_uint8, CV_8U, 255);

        blend_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat dilated, eroded;
        cv::dilate(front_mask_uint8, dilated, blend_kernel_, cv::Point(-1, -1), 2);
        cv::erode(front_mask_uint8, eroded, blend_kernel_, cv::Point(-1, -1), 2);
        front_edge_ = dilated - eroded;

        cv::distanceTransform(front_mask_uint8, front_distance_, cv::DIST_L2, cv::DIST_MASK_5);
        front_distance_ *= 0.3f;
        cv::min(front_distance_, 1.0f, front_distance_);

        maps_initialized_ = true;
    }

    cv::Mat create_equirectangular(const cv::Mat& dual_fisheye_bgr)
    {
        const int img_height = dual_fisheye_bgr.rows;
        const int midpoint = dual_fisheye_bgr.cols / 2;
        cv::Mat front_img_full = dual_fisheye_bgr(cv::Rect(midpoint, 0, midpoint, img_height));
        cv::Mat back_img_full = dual_fisheye_bgr(cv::Rect(0, 0, midpoint, img_height));

        cv::Mat front_img;
        cv::Mat back_img;
        const int orig_h = front_img_full.rows;
        const int orig_w = front_img_full.cols;

        if (orig_h != crop_size_ || orig_w != crop_size_) {
            const int y_start = (orig_h - crop_size_) / 2;
            const int x_start = (orig_w - crop_size_) / 2;
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

        if (!maps_initialized_ || front_img.rows != img_height_ || front_img.cols != img_width_) {
            const auto map_init_start = std::chrono::steady_clock::now();
            init_mapping(front_img.rows, front_img.cols);
            map_init_time_us_acc_ += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - map_init_start).count());
        }

        const auto remap_start = std::chrono::steady_clock::now();
        cv::Mat front_result;
        cv::Mat back_result;
        cv::remap(front_img, front_result, front_map_x_, front_map_y_, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar(0, 0, 0));
        cv::remap(back_img, back_result, back_map_x_, back_map_y_, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar(0, 0, 0));
        cv::Mat equirect = cv::Mat::zeros(crop_out_height_, out_width_, CV_8UC3);

        if (enable_seam_blending_) {
            for (int y = 0; y < crop_out_height_; ++y) {
                for (int x = 0; x < out_width_; ++x) {
                    const uchar edge_val = front_edge_.at<uchar>(y, x);

                    if (edge_val == 0) {
                        if (front_mask_.at<uchar>(y, x)) {
                            equirect.at<cv::Vec3b>(y, x) = front_result.at<cv::Vec3b>(y, x);
                        } else {
                            equirect.at<cv::Vec3b>(y, x) = back_result.at<cv::Vec3b>(y, x);
                        }
                    } else {
                        const float alpha = front_distance_.at<float>(y, x);
                        const cv::Vec3f f = cv::Vec3f(front_result.at<cv::Vec3b>(y, x));
                        const cv::Vec3f b = cv::Vec3f(back_result.at<cv::Vec3b>(y, x));
                        const cv::Vec3f blended = alpha * f + (1.0f - alpha) * b;

                        equirect.at<cv::Vec3b>(y, x) = cv::Vec3b(
                            cv::saturate_cast<uchar>(blended[0]),
                            cv::saturate_cast<uchar>(blended[1]),
                            cv::saturate_cast<uchar>(blended[2])
                        );
                    }
                }
            }
        } else {
            front_result.copyTo(equirect, front_mask_);
            back_result.copyTo(equirect, back_mask_);
        }

        remap_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - remap_start).count());
        return equirect;
    }

    void init_decoder()
    {
        hw_type_ = gpu_enabled_ ? AV_HWDEVICE_TYPE_CUDA : AV_HWDEVICE_TYPE_NONE;
        const char* decoder_name = "h264_cuvid";

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            codec_ = avcodec_find_decoder_by_name(decoder_name);
        }
        if (!codec_) {
            hw_type_ = AV_HWDEVICE_TYPE_NONE;
            codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
        }
        if (!codec_) {
            throw std::runtime_error("No H.264 decoder available");
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE) {
            if (av_hwdevice_ctx_create(&hw_device_ctx_, hw_type_, nullptr, nullptr, 0) < 0) {
                hw_type_ = AV_HWDEVICE_TYPE_NONE;
            }
        }

        parser_ctx_ = av_parser_init(codec_->id);
        codec_ctx_ = avcodec_alloc_context3(codec_);
        pkt_ = av_packet_alloc();
        hw_frame_ = av_frame_alloc();

        if (!parser_ctx_ || !codec_ctx_ || !pkt_ || !hw_frame_) {
            throw std::runtime_error("Failed to allocate decoder state");
        }

        if (hw_type_ != AV_HWDEVICE_TYPE_NONE && hw_device_ctx_) {
            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->get_format = get_hw_format;
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                throw std::runtime_error("Failed to allocate software frame");
            }
        }

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
            throw std::runtime_error("Failed to open decoder");
        }
    }

    void cleanup_decoder()
    {
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

    void maybe_log_performance()
    {
        const auto now_tp = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - perf_window_start_).count();
        if (elapsed_ms < 2000) {
            return;
        }

        const double sec = static_cast<double>(elapsed_ms) / 1000.0;
        const double decoded_fps = sec > 0.0 ? static_cast<double>(decoded_frame_counter_) / sec : 0.0;
        const double published_fps = sec > 0.0 ? static_cast<double>(published_frame_counter_) / sec : 0.0;
        const double avg_decode_ms = decoded_frame_counter_ > 0
            ? static_cast<double>(decode_time_us_acc_) / static_cast<double>(decoded_frame_counter_) / 1000.0
            : 0.0;
        const double avg_map_init_ms = published_frame_counter_ > 0
            ? static_cast<double>(map_init_time_us_acc_) / static_cast<double>(published_frame_counter_) / 1000.0
            : 0.0;
        const double avg_remap_ms = published_frame_counter_ > 0
            ? static_cast<double>(remap_time_us_acc_) / static_cast<double>(published_frame_counter_) / 1000.0
            : 0.0;
        const double avg_publish_ms = published_frame_counter_ > 0
            ? static_cast<double>(publish_time_us_acc_) / static_cast<double>(published_frame_counter_) / 1000.0
            : 0.0;
        const double avg_total_ms = published_frame_counter_ > 0
            ? static_cast<double>(total_time_us_acc_) / static_cast<double>(published_frame_counter_) / 1000.0
            : 0.0;

        RCLCPP_INFO(
            get_logger(),
            "Compressed equirect stats: decoded=%.1f fps, published=%.1f fps, avg_decode=%.2f ms, avg_map_init=%.2f ms, avg_remap=%.2f ms, avg_publish=%.2f ms, avg_total=%.2f ms",
            decoded_fps,
            published_fps,
            avg_decode_ms,
            avg_map_init_ms,
            avg_remap_ms,
            avg_publish_ms,
            avg_total_ms);

        decoded_frame_counter_ = 0;
        published_frame_counter_ = 0;
        decode_time_us_acc_ = 0;
        map_init_time_us_acc_ = 0;
        remap_time_us_acc_ = 0;
        publish_time_us_acc_ = 0;
        total_time_us_acc_ = 0;
        perf_window_start_ = now_tp;
    }

    void publish_equirect(const cv::Mat& equirect, const std_msgs::msg::Header& header)
    {
        const auto publish_start = std::chrono::steady_clock::now();
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header = header;
        msg->height = static_cast<uint32_t>(equirect.rows);
        msg->width = static_cast<uint32_t>(equirect.cols);
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = static_cast<uint32_t>(equirect.step[0]);
        const size_t size = static_cast<size_t>(equirect.rows) * equirect.step[0];
        msg->data.assign(equirect.data, equirect.data + size);
        equirect_pub_->publish(std::move(msg));
        publish_camera_info(header, equirect.rows, equirect.cols);
        publish_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - publish_start).count());
        ++published_frame_counter_;
    }

    void publish_camera_info(const std_msgs::msg::Header& header, int height, int width)
    {
        auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
        msg->header = header;
        msg->height = static_cast<uint32_t>(height);
        msg->width = static_cast<uint32_t>(width);
        camera_info_pub_->publish(std::move(msg));
    }

    void handle_decoded_frame(AVFrame* frame, const std_msgs::msg::Header& header)
    {
        const auto total_start = std::chrono::steady_clock::now();
        AVFrame* frame_to_use = frame;

        if (frame->format == AV_PIX_FMT_CUDA || frame->format == AV_PIX_FMT_VAAPI) {
            if (!sw_frame_ || av_hwframe_transfer_data(sw_frame_, frame, 0) < 0) {
                av_frame_unref(frame);
                return;
            }
            frame_to_use = sw_frame_;
        }

        const auto decode_start = std::chrono::steady_clock::now();
        bool converted = false;
        if (frame_to_use->format == AV_PIX_FMT_NV12 && frame_to_use->data[0] && frame_to_use->data[1]) {
            const int width = frame_to_use->width;
            const int height = frame_to_use->height;
            cv::Mat y_plane(height, width, CV_8UC1, frame_to_use->data[0], frame_to_use->linesize[0]);
            cv::Mat uv_plane(height / 2, width / 2, CV_8UC2, frame_to_use->data[1], frame_to_use->linesize[1]);
            cv::cvtColorTwoPlane(y_plane, uv_plane, bgr_frame_, cv::COLOR_YUV2BGR_NV12);
            converted = !bgr_frame_.empty();
        } else if (frame_to_use->format == AV_PIX_FMT_BGR24 && frame_to_use->data[0]) {
            const int width = frame_to_use->width;
            const int height = frame_to_use->height;
            bgr_frame_.create(height, width, CV_8UC3);
            for (int y = 0; y < height; ++y) {
                std::memcpy(
                    bgr_frame_.ptr(y),
                    frame_to_use->data[0] + static_cast<size_t>(y) * static_cast<size_t>(frame_to_use->linesize[0]),
                    static_cast<size_t>(width) * 3);
            }
            converted = true;
        }

        if (!converted) {
            if (!sws_ctx_ && frame_to_use->width > 0 && frame_to_use->height > 0) {
                sws_ctx_ = sws_getContext(
                    frame_to_use->width,
                    frame_to_use->height,
                    static_cast<AVPixelFormat>(frame_to_use->format),
                    frame_to_use->width,
                    frame_to_use->height,
                    AV_PIX_FMT_BGR24,
                    SWS_FAST_BILINEAR,
                    nullptr,
                    nullptr,
                    nullptr);
                if (sws_ctx_) {
                    bgr_frame_.create(frame_to_use->height, frame_to_use->width, CV_8UC3);
                }
            }
            if (sws_ctx_ && !bgr_frame_.empty()) {
                uint8_t* dst_data[4] = {bgr_frame_.data, nullptr, nullptr, nullptr};
                int dst_linesize[4] = {static_cast<int>(bgr_frame_.step[0]), 0, 0, 0};
                sws_scale(
                    sws_ctx_,
                    (const uint8_t* const*)frame_to_use->data,
                    frame_to_use->linesize,
                    0,
                    frame_to_use->height,
                    dst_data,
                    dst_linesize);
                converted = true;
            }
        }

        decode_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - decode_start).count());
        ++decoded_frame_counter_;

        if (converted && !bgr_frame_.empty()) {
            cv::Mat equirect = create_equirectangular(bgr_frame_);
            publish_equirect(equirect, header);
            total_time_us_acc_ += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - total_start).count());
            maybe_log_performance();
        }

        av_frame_unref(frame);
        if (frame_to_use == sw_frame_) {
            av_frame_unref(sw_frame_);
        }
    }

    void decode_packet(AVPacket* packet, const std_msgs::msg::Header& header)
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

            if (skip_frame_ > 0 && !i_frame_only_) {
                const bool should_process = (frame_skip_counter_++ % (skip_frame_ + 1) == 0);
                if (!should_process) {
                    av_frame_unref(hw_frame_);
                    continue;
                }
            }

            handle_decoded_frame(hw_frame_, header);
        }
    }

    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (msg->format != "h264" || !codec_ctx_ || !parser_ctx_ || !pkt_ || !hw_frame_) {
            return;
        }

        std::lock_guard<std::mutex> lock(decoder_mutex_);
        const uint8_t* cur_data = msg->data.data();
        size_t remaining_size = msg->data.size();

        while (remaining_size > 0) {
            const int bytes_parsed = av_parser_parse2(
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
                av_packet_unref(pkt_);
                continue;
            }

            decode_packet(pkt_, msg->header);
            av_packet_unref(pkt_);
        }
    }

    std::string compressed_topic_;
    std::string output_topic_;
    std::string camera_info_topic_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr equirect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

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
    std::mutex decoder_mutex_;

    int skip_frame_ = 0;
    uint64_t frame_skip_counter_ = 0;
    bool i_frame_only_ = false;

    double cx_offset_;
    double cy_offset_;
    double front_cx_offset_;
    double front_cy_offset_;
    double back_cx_offset_;
    double back_cy_offset_;
    int crop_size_;
    double tx_, ty_, tz_;
    double roll_, pitch_, yaw_;
    bool gpu_enabled_;
    int out_width_;
    int out_height_;
    int crop_out_height_;
    bool enable_seam_blending_ = false;

    cv::Mat back_to_front_rotation_;
    cv::Vec3d back_to_front_translation_;
    cv::Mat front_map_x_, front_map_y_;
    cv::Mat back_map_x_, back_map_y_;
    cv::Mat front_mask_, back_mask_;
    cv::Mat front_edge_;
    cv::Mat front_distance_;
    cv::Mat blend_kernel_;
    bool maps_initialized_ = false;
    int img_height_ = 0;
    int img_width_ = 0;

    uint64_t decoded_frame_counter_ = 0;
    uint64_t published_frame_counter_ = 0;
    uint64_t decode_time_us_acc_ = 0;
    uint64_t map_init_time_us_acc_ = 0;
    uint64_t remap_time_us_acc_ = 0;
    uint64_t publish_time_us_acc_ = 0;
    uint64_t total_time_us_acc_ = 0;
    std::chrono::steady_clock::time_point perf_window_start_ = std::chrono::steady_clock::now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EquirectangularCropH264Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
