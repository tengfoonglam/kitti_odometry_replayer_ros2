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

/**
 * @brief Load image given a provided path
 *
 * @param image_path - Path to load image from
 * @return ImageMsg::SharedPtr - Loaded image. Empty pointer if failed
 */
[[nodiscard]] ImageMsg::SharedPtr load_image_from_file(const std::filesystem::path & image_path);

/**
 * @brief Check if an image file adheres to KITTI naming convention
 *
 * @param image_path - Path to load image from
 * @return true - Is KITTI image file
 * @return false - Otherwise
 */
[[nodiscard]] bool is_kitti_image_file(const std::filesystem::path & image_path);

/**
 * @brief Given an index and path, get the full path of the image with the correct KITTI naming
 * convension and file extension
 *
 * @param idx
 * @param folder_path
 * @return std::filesystem::path - Resulting path
 */
[[nodiscard]] std::filesystem::path from_index_to_image_file_path(
  std::size_t idx, const std::filesystem::path & folder_path);

/**
 * @brief In a folder, find the image file with the largest index
 *
 * @param load_path - Folder to look for the file
 * @return std::optional<size_t> - Largest index, if available
 */
[[nodiscard]] std::optional<std::size_t> get_last_index_of_image_sequence(
  const std::filesystem::path & image_path);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_
