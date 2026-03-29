#ifndef EQUIRECTANGULAR_CROP_HPP
#define EQUIRECTANGULAR_CROP_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

class EquirectangularCropNode : public rclcpp::Node
{
public:
    explicit EquirectangularCropNode();
    ~EquirectangularCropNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void loadParameters();
    void updateCameraParameters();
    void initMapping(int img_height, int img_width);
    cv::Mat createEquirectangular(const cv::Mat& front_img, const cv::Mat& back_img);
    void maybeLogPerformance();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr dual_fisheye_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr equirect_pub_;

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

    cv::Mat back_to_front_rotation_;
    cv::Vec3d back_to_front_translation_;

    cv::Mat front_map_x_, front_map_y_;
    cv::Mat back_map_x_, back_map_y_;
    cv::Mat front_mask_, back_mask_;

    std::atomic<bool> maps_initialized_;
    std::atomic<bool> params_changed_;
    int img_height_;
    int img_width_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    uint64_t processed_frame_counter_ = 0;
    uint64_t convert_time_us_acc_ = 0;
    uint64_t crop_time_us_acc_ = 0;
    uint64_t map_init_time_us_acc_ = 0;
    uint64_t remap_time_us_acc_ = 0;
    uint64_t publish_time_us_acc_ = 0;
    uint64_t total_time_us_acc_ = 0;
    std::chrono::steady_clock::time_point perf_window_start_ = std::chrono::steady_clock::now();

    std::mutex processing_mutex_;
};

#endif // EQUIRECTANGULAR_CROP_HPP
