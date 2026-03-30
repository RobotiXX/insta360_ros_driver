#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <rclcpp/rclcpp.hpp>

class OpenCvCudaCheckNode : public rclcpp::Node {
public:
  OpenCvCudaCheckNode() : Node("opencv_cuda_check") {
    const std::string build_info = cv::getBuildInformation();

    RCLCPP_INFO(get_logger(), "OpenCV version: %s", CV_VERSION);

    const bool cuda_built = contains_enabled_flag(build_info, "NVIDIA CUDA:");
    const bool cudnn_built = contains_enabled_flag(build_info, "cuDNN:");
    const int cuda_device_count = cv::cuda::getCudaEnabledDeviceCount();

    RCLCPP_INFO(get_logger(), "OpenCV built with CUDA: %s", cuda_built ? "YES" : "NO");
    RCLCPP_INFO(get_logger(), "OpenCV built with cuDNN: %s", cudnn_built ? "YES" : "NO");
    RCLCPP_INFO(get_logger(), "CUDA-enabled devices visible to OpenCV: %d", cuda_device_count);

    if (cuda_built && cuda_device_count > 0) {
      cv::cuda::DeviceInfo device_info(0);
      RCLCPP_INFO(
        get_logger(),
        "First CUDA device: %s, compute capability %d.%d, total memory %.2f GiB",
        device_info.name(),
        device_info.majorVersion(),
        device_info.minorVersion(),
        static_cast<double>(device_info.totalMemory()) / (1024.0 * 1024.0 * 1024.0));
      RCLCPP_INFO(get_logger(), "OpenCV CUDA support is available and usable.");
      result_code_ = 0;
      return;
    }

    if (cuda_built && cuda_device_count == 0) {
      RCLCPP_WARN(
        get_logger(),
        "OpenCV was built with CUDA support, but no CUDA-capable device is currently usable.");
      result_code_ = 1;
      return;
    }

    RCLCPP_WARN(get_logger(), "OpenCV was not built with CUDA support.");
    result_code_ = 1;
  }

  int result_code() const { return result_code_; }

private:
  static bool contains_enabled_flag(const std::string &build_info, const std::string &key) {
    std::istringstream stream(build_info);
    std::string line;

    while (std::getline(stream, line)) {
      if (line.find(key) == std::string::npos) {
        continue;
      }

      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }

      std::string value = line.substr(colon + 1);
      std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });

      return value.find("YES") != std::string::npos;
    }

    return false;
  }

  int result_code_{1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCvCudaCheckNode>();
  const int result = node->result_code();
  rclcpp::shutdown();
  return result;
}
