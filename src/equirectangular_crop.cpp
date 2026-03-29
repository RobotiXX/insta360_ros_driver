#include "equirectangular_crop.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <cmath>

EquirectangularCropNode::EquirectangularCropNode()
    : Node("equirectangular_crop_node"),
      maps_initialized_(false),
      params_changed_(true),
      img_height_(0),
      img_width_(0)
{
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

    loadParameters();

    RCLCPP_INFO(get_logger(), "C++ cropped equirectangular node");

    params_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&EquirectangularCropNode::parametersCallback, this, std::placeholders::_1));

    updateCameraParameters();

    auto qos = rclcpp::QoS(10).best_effort();

    dual_fisheye_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/dual_fisheye/image", qos,
        std::bind(&EquirectangularCropNode::imageCallback, this, std::placeholders::_1));

    equirect_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/equirectangular/image", qos);
}

EquirectangularCropNode::~EquirectangularCropNode()
{
}

void EquirectangularCropNode::loadParameters()
{
    try {
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

        auto translation = get_parameter("translation").as_double_array();
        tx_ = translation[0];
        ty_ = translation[1];
        tz_ = translation[2];

        auto rotation_deg = get_parameter("rotation_deg").as_double_array();
        roll_ = rotation_deg[0] * M_PI / 180.0;
        pitch_ = rotation_deg[1] * M_PI / 180.0;
        yaw_ = rotation_deg[2] * M_PI / 180.0;

        if (crop_out_height_ <= 0) {
            RCLCPP_WARN(get_logger(), "crop_out_height must be positive, clamping to 1");
            crop_out_height_ = 1;
        }
        if (crop_out_height_ > out_height_) {
            RCLCPP_WARN(
                get_logger(),
                "crop_out_height (%d) exceeds out_height (%d), clamping to out_height",
                crop_out_height_,
                out_height_);
            crop_out_height_ = out_height_;
        }

        RCLCPP_INFO(get_logger(), "Loaded parameters from ROS parameter server");
        RCLCPP_INFO(get_logger(), "  Crop size: %d", crop_size_);
        RCLCPP_INFO(get_logger(), "  Shared center offset: (%.1f, %.1f)", cx_offset_, cy_offset_);
        RCLCPP_INFO(get_logger(), "  Front center offset: (%.1f, %.1f)", front_cx_offset_, front_cy_offset_);
        RCLCPP_INFO(get_logger(), "  Back center offset: (%.1f, %.1f)", back_cx_offset_, back_cy_offset_);
        RCLCPP_INFO(get_logger(), "  Translation: [%.3f, %.3f, %.3f]", tx_, ty_, tz_);
        RCLCPP_INFO(get_logger(), "  Rotation (deg): [%.1f, %.1f, %.1f]",
                    rotation_deg[0], rotation_deg[1], rotation_deg[2]);
        RCLCPP_INFO(get_logger(), "  Virtual output size: %dx%d", out_width_, out_height_);
        RCLCPP_INFO(get_logger(), "  Published cropped output size: %dx%d", out_width_, crop_out_height_);
        RCLCPP_INFO(get_logger(), "  GPU enabled: %s", gpu_enabled_ ? "true" : "false");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error loading parameters: %s", e.what());
        gpu_enabled_ = true;
        throw;
    }
}

void EquirectangularCropNode::updateCameraParameters()
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

    if (maps_initialized_) {
        maps_initialized_ = false;
        RCLCPP_INFO(get_logger(), "Parameters updated, remapping will occur on next image");
    }
}

void EquirectangularCropNode::initMapping(int img_height, int img_width)
{
    RCLCPP_INFO(
        get_logger(),
        "Initializing cropped equirectangular projection: fusing two %dx%d fisheye images to %dx%d from virtual %dx%d",
        img_width, img_height, out_width_, crop_out_height_, out_width_, out_height_);

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

    maps_initialized_ = true;

    RCLCPP_INFO(get_logger(), "Cropped mapping matrices initialization complete");
}

cv::Mat EquirectangularCropNode::createEquirectangular(const cv::Mat& front_img, const cv::Mat& back_img)
{
    if (!maps_initialized_ || params_changed_ ||
        front_img.rows != img_height_ || front_img.cols != img_width_) {
        initMapping(front_img.rows, front_img.cols);
        params_changed_ = false;
    }

    if (!maps_initialized_) {
        RCLCPP_ERROR(get_logger(), "Mapping arrays not properly initialized");
        return cv::Mat::zeros(crop_out_height_, out_width_, CV_8UC3);
    }

    cv::Mat front_result, back_result;
    cv::remap(front_img, front_result, front_map_x_, front_map_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::remap(back_img, back_result, back_map_x_, back_map_y_, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    cv::Mat equirect = cv::Mat::zeros(crop_out_height_, out_width_, CV_8UC3);
    front_result.copyTo(equirect, front_mask_);
    back_result.copyTo(equirect, back_mask_);
    return equirect;
}

void EquirectangularCropNode::maybeLogPerformance()
{
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - perf_window_start_).count();
    if (elapsed_ms < 2000) {
        return;
    }

    const double sec = static_cast<double>(elapsed_ms) / 1000.0;
    const double processed_fps = sec > 0.0 ? static_cast<double>(processed_frame_counter_) / sec : 0.0;
    const double avg_convert_ms = processed_frame_counter_ > 0
        ? static_cast<double>(convert_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;
    const double avg_crop_ms = processed_frame_counter_ > 0
        ? static_cast<double>(crop_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;
    const double avg_map_init_ms = processed_frame_counter_ > 0
        ? static_cast<double>(map_init_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;
    const double avg_remap_ms = processed_frame_counter_ > 0
        ? static_cast<double>(remap_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;
    const double avg_publish_ms = processed_frame_counter_ > 0
        ? static_cast<double>(publish_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;
    const double avg_total_ms = processed_frame_counter_ > 0
        ? static_cast<double>(total_time_us_acc_) / static_cast<double>(processed_frame_counter_) / 1000.0
        : 0.0;

    RCLCPP_INFO(
        get_logger(),
        "Equirect crop stats: processed=%.1f fps, avg_total=%.2f ms, avg_convert=%.2f ms, avg_crop=%.2f ms, avg_map_init=%.2f ms, avg_remap=%.2f ms, avg_publish=%.2f ms",
        processed_fps,
        avg_total_ms,
        avg_convert_ms,
        avg_crop_ms,
        avg_map_init_ms,
        avg_remap_ms,
        avg_publish_ms);

    processed_frame_counter_ = 0;
    convert_time_us_acc_ = 0;
    crop_time_us_acc_ = 0;
    map_init_time_us_acc_ = 0;
    remap_time_us_acc_ = 0;
    publish_time_us_acc_ = 0;
    total_time_us_acc_ = 0;
    perf_window_start_ = now_tp;
}

void EquirectangularCropNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr dual_fisheye_msg)
{
    try {
        const auto total_start = std::chrono::steady_clock::now();

        const auto convert_start = std::chrono::steady_clock::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(dual_fisheye_msg, "rgb8");
        cv::Mat dual_fisheye_img = cv_ptr->image;
        const auto convert_end = std::chrono::steady_clock::now();

        const int img_height = dual_fisheye_img.rows;
        const int img_width_full = dual_fisheye_img.cols;
        const int midpoint = img_width_full / 2;

        const auto crop_start = std::chrono::steady_clock::now();
        cv::Mat front_img_full = dual_fisheye_img(cv::Rect(midpoint, 0, midpoint, img_height));
        cv::Mat back_img_full = dual_fisheye_img(cv::Rect(0, 0, midpoint, img_height));

        cv::Mat front_img, back_img;
        const int current_crop_size = crop_size_;
        const int orig_h = front_img_full.rows;
        const int orig_w = front_img_full.cols;

        if (orig_h != current_crop_size || orig_w != current_crop_size) {
            const int y_start = (orig_h - current_crop_size) / 2;
            const int x_start = (orig_w - current_crop_size) / 2;

            if (y_start >= 0 && x_start >= 0 &&
                y_start + current_crop_size <= orig_h &&
                x_start + current_crop_size <= orig_w) {
                front_img = front_img_full(cv::Rect(x_start, y_start, current_crop_size, current_crop_size));
                back_img = back_img_full(cv::Rect(x_start, y_start, current_crop_size, current_crop_size));
            } else {
                front_img = front_img_full;
                back_img = back_img_full;
            }
        } else {
            front_img = front_img_full;
            back_img = back_img_full;
        }
        const auto crop_end = std::chrono::steady_clock::now();

        uint64_t map_init_us = 0;
        if (!maps_initialized_ || params_changed_ ||
            front_img.rows != img_height_ || front_img.cols != img_width_) {
            const auto map_init_start = std::chrono::steady_clock::now();
            initMapping(front_img.rows, front_img.cols);
            const auto map_init_end = std::chrono::steady_clock::now();
            map_init_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(map_init_end - map_init_start).count());
            params_changed_ = false;
        }

        const auto remap_start = std::chrono::steady_clock::now();
        cv::Mat equirect_img = createEquirectangular(front_img, back_img);
        const auto remap_end = std::chrono::steady_clock::now();

        const auto publish_start = std::chrono::steady_clock::now();
        cv_bridge::CvImage out_msg;
        out_msg.header = dual_fisheye_msg->header;
        out_msg.encoding = "rgb8";
        out_msg.image = equirect_img;
        equirect_pub_->publish(*out_msg.toImageMsg());
        const auto publish_end = std::chrono::steady_clock::now();

        const auto total_end = std::chrono::steady_clock::now();
        convert_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(convert_end - convert_start).count());
        crop_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(crop_end - crop_start).count());
        map_init_time_us_acc_ += map_init_us;
        remap_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(remap_end - remap_start).count());
        publish_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(publish_end - publish_start).count());
        total_time_us_acc_ += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count());
        ++processed_frame_counter_;
        maybeLogPerformance();
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error processing images: %s", e.what());
    }
}

rcl_interfaces::msg::SetParametersResult EquirectangularCropNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    bool update_needed = false;

    for (const auto& param : parameters) {
        if (param.get_name() == "cx_offset" ||
            param.get_name() == "cy_offset" ||
            param.get_name() == "front_cx_offset" ||
            param.get_name() == "front_cy_offset" ||
            param.get_name() == "back_cx_offset" ||
            param.get_name() == "back_cy_offset" ||
            param.get_name() == "crop_size" ||
            param.get_name() == "translation" ||
            param.get_name() == "rotation_deg" ||
            param.get_name() == "out_width" ||
            param.get_name() == "out_height" ||
            param.get_name() == "crop_out_height" ||
            param.get_name() == "gpu") {
            update_needed = true;
        }
    }

    if (update_needed) {
        loadParameters();
        updateCameraParameters();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EquirectangularCropNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
