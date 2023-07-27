#include "ros2_kitti_core/image_utils.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/data_utils.hpp"

namespace r2k_core
{

ImageMsg::SharedPtr load_image_from_file(const std::filesystem::path & image_path)
{
  // Check if text file is .png file and exists
  if (!file_exists_and_correct_extension(image_path, std::string{kKittiImageExtension})) {
    return ImageMsg::SharedPtr();
  }

  // Read image and convert to message
  const auto img = cv::imread(image_path, cv::IMREAD_UNCHANGED);

  if (img.empty()) {
    return ImageMsg::SharedPtr();
  }

  const bool is_gray_image = (img.type() == kGrayImageOpenCVType);
  const bool is_colour_image = (img.type() == kColourImageOpenCVType);
  if (!(is_gray_image || is_colour_image)) {
    return ImageMsg::SharedPtr();
  }

  const auto encoding =
    is_gray_image ? std::string{kGrayImageEncoding} : std::string{kColourImageEncoding};

  return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
}

bool is_kitti_image_file(const std::filesystem::path & image_path)
{
  return is_numbered_file_with_correction_extension(
    image_path, kNumberDigitsImageFilename, std::string{kKittiImageExtension});
}

std::filesystem::path from_index_to_image_file_path(
  std::size_t idx, const std::filesystem::path & folder_path)
{
  return from_index_to_file_path(
    idx, folder_path, kNumberDigitsImageFilename, std::string{kKittiImageExtension});
}

std::optional<std::size_t> get_last_index_of_image_sequence(
  const std::filesystem::path & image_path)
{
  return get_last_index_of_data_sequence(
    image_path, kNumberDigitsImageFilename, std::string{kKittiImageExtension});
}

}  // namespace r2k_core
