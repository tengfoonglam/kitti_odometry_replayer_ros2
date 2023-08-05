#ifndef ROS2_KITTI_CORE_TEST__TEST_WITH_POINT_CLOUD_IO_HPP_
#define ROS2_KITTI_CORE_TEST__TEST_WITH_POINT_CLOUD_IO_HPP_

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <ros2_kitti_core/point_cloud_utils.hpp>
#include <string>
#include <vector>

#include "ros2_kitti_core_test/test_with_io.hpp"

namespace r2k_core_test
{

class TestWithPointCloudIO : public r2k_core_test::TestWithIO
{
public:
  using KITTIPoint = std::array<float, 4>;
  using KITTIPoints = std::vector<KITTIPoint>;

  static const KITTIPoints kTestPoints;

  static void write_kitti_bin_files(
    const std::vector<std::size_t> & indices, const std::filesystem::path & folder,
    const std::vector<KITTIPoints> & point_clouds)
  {
    ASSERT_EQ(indices.size(), point_clouds.size());

    for (std::size_t i = 0; i < indices.size(); i++) {
      const auto file_path = r2k_core::from_index_to_point_cloud_file_path(indices.at(i), folder);
      write_bin_file(file_path, point_clouds.at(i));
    }
  }

  static void write_bin_file(const std::filesystem::path & file_path, const KITTIPoints & points)
  {
    const auto number_points = points.size();
    constexpr auto number_fields = KITTIPoint{}.size();
    std::vector<KITTIPoint::value_type> data;
    data.reserve(number_points * number_fields);
    for (const auto & point : points) {
      data.insert(std::end(data), std::cbegin(point), std::cend(point));
    }

    // Example code as provided by the KITTI README
    auto * stream_ptr = fopen(file_path.string().c_str(), "wb");
    if (!points.empty()) {
      auto * data_ptr = reinterpret_cast<const char *>(&data[0]);
      fwrite(data_ptr, sizeof(KITTIPoint::value_type), number_fields * number_points, stream_ptr);
    }
    fclose(stream_ptr);
    ASSERT_TRUE(std::filesystem::exists(file_path));
  }
};
const TestWithPointCloudIO::KITTIPoints TestWithPointCloudIO::kTestPoints{
  {1.0f, 2.0f, 3.0f, 4.0f},
  {5.0f, 6.0f, 7.0f, 8.0f},
  {9.0f, 10.0f, 11.0f, 12.0f},
  {13.0f, 14.0f, 15.0f, 16.0f},
  {17.0f, 18.0f, 19.0f, 20.0f}};
}  // namespace r2k_core_test

#endif  // ROS2_KITTI_CORE_TEST__TEST_WITH_POINT_CLOUD_IO_HPP_
