#ifndef ROS2_KITTI_CORE_TEST__TEST_WITH_IMAGE_IO_HPP_
#define ROS2_KITTI_CORE_TEST__TEST_WITH_IMAGE_IO_HPP_

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "ros2_kitti_core_test/test_with_io.hpp"

namespace r2k_core_test
{

class TestWithImageIO : public r2k_core_test::TestWithIO
{
public:
  static constexpr int kSampleImageWidth{20};
  static constexpr int kSampleImageHeight{10};

  static void write_example_image_files(
    const std::vector<std::size_t> & indices, const std::filesystem::path & folder, int image_type)
  {
    for (std::size_t i = 0; i < indices.size(); i++) {
      const auto file_path = r2k_core::from_index_to_image_file_path(indices.at(i), folder);
      write_sample_image(file_path, image_type);
    }
  }

  static void write_sample_image(const std::filesystem::path & file_path, const int image_type)
  {
    const cv::Mat img(cv::Size(kSampleImageWidth, kSampleImageHeight), image_type);
    cv::imwrite(file_path.string(), img);
    ASSERT_TRUE(std::filesystem::exists(file_path));
  }
};

}  // namespace r2k_core_test

#endif  // ROS2_KITTI_CORE_TEST__TEST_WITH_IMAGE_IO_HPP_
