#ifndef ROS2_KITTI_CORE__IMAGE_UTILS_HPP_
#define ROS2_KITTI_CORE__IMAGE_UTILS_HPP_

#include <filesystem>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace r2k_core
{

using ImageMsg = sensor_msgs::msg::Image;

static constexpr const char kGrayImageEncoding[]{"mono8"};
static constexpr const char kColourImageEncoding[]{"rgb8"};
static constexpr std::size_t kNumberDigitsImageFilename{6};
static constexpr const char kKittiImageExtension[]{".png"};

[[nodiscard]] ImageMsg::SharedPtr load_image_from_file(const std::filesystem::path & image_path);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_
