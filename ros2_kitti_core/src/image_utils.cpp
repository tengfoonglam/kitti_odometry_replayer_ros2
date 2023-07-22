#include "ros2_kitti_core/image_utils.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace r2k_core
{

[[nodiscard]] ImageMsg::SharedPtr load_image_from_file(const std::filesystem::path & image_path)
{
  // Check if text file is .png file and exists
  if (
    !std::filesystem::exists(image_path) ||
    image_path.extension().string() != std::string{kKittiImageExtension}) {
    return ImageMsg::SharedPtr();
  }

  // Read image and convert to message
  const auto img = cv::imread(image_path, cv::IMREAD_UNCHANGED);

  if (img.empty()) {
    return ImageMsg::SharedPtr();
  }

  const bool is_gray_image = (img.channels() == 1);
  const bool is_colour_image = (img.channels() == 3);
  if (!(is_gray_image || is_colour_image)) {
    return ImageMsg::SharedPtr();
  }

  const auto encoding =
    is_gray_image ? std::string{kGrayImageEncoding} : std::string{kColourImageEncoding};

  return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
}

}  // namespace r2k_core
