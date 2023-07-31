#ifndef ROS2_KITTI_CORE__IMAGE_UTILS_HPP_
#define ROS2_KITTI_CORE__IMAGE_UTILS_HPP_

#include <filesystem>
#include <opencv2/core.hpp>
#include <optional>
#include <sensor_msgs/msg/image.hpp>

namespace r2k_core
{

using ImageMsg = sensor_msgs::msg::Image;

static constexpr const char kGrayImageEncoding[]{"mono8"};
static constexpr auto kGrayImageOpenCVType{CV_8UC1};
static constexpr const char kColourImageEncoding[]{"bgr8"};
static constexpr auto kColourImageOpenCVType{CV_8UC3};
static constexpr std::size_t kNumberDigitsImageFilename{6};
static constexpr const char kKittiImageExtension[]{".png"};

[[nodiscard]] ImageMsg::SharedPtr load_image_from_file(const std::filesystem::path & image_path);

[[nodiscard]] bool is_kitti_image_file(const std::filesystem::path & image_path);

[[nodiscard]] std::filesystem::path from_index_to_image_file_path(
  std::size_t idx, const std::filesystem::path & folder_path);

[[nodiscard]] std::optional<std::size_t> get_last_index_of_image_sequence(
  const std::filesystem::path & image_path);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_
