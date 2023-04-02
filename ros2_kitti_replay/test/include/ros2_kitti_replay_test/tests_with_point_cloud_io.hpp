#include <gtest/gtest.h>
#ifndef ROS2_KITTI_REPLAY_TEST__TESTS_WITH_POINT_CLOUD_IO_HPP_
#define ROS2_KITTI_REPLAY_TEST__TESTS_WITH_POINT_CLOUD_IO_HPP_

#include <array>
#include <filesystem>
#include <ros2_kitti_replay/point_cloud_utils.hpp>
#include <string>
#include <vector>

namespace r2k_replay_test
{

class TestsWithPointCloudIO : public ::testing::Test
{
public:
  using KITTIPoint = std::array<float, 4>;
  using KITTIPoints = std::vector<KITTIPoint>;

  static const std::filesystem::path kTestFolderPath;
  static const KITTIPoints kTestPoints;

  static void write_bin_file(const std::filesystem::path & file_path, const KITTIPoints & points)
  {
    const auto number_points = points.size();
    constexpr auto number_fields = KITTIPoint{}.size();
    std::vector<KITTIPoint::value_type> data;
    data.reserve(number_points * number_fields);
    for (const auto & point : points) {
      data.insert(data.end(), point.cbegin(), point.cend());
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

protected:
  void SetUp() { std::filesystem::create_directory(kTestFolderPath); }
  void TearDown() { std::filesystem::remove_all(kTestFolderPath); }
};
const TestsWithPointCloudIO::KITTIPoints TestsWithPointCloudIO::kTestPoints{
  {1.0f, 2.0f, 3.0f, 4.0f},
  {5.0f, 6.0f, 7.0f, 8.0f},
  {9.0f, 10.0f, 11.0f, 12.0f},
  {13.0f, 14.0f, 15.0f, 16.0f},
  {17.0f, 18.0f, 19.0f, 20.0f}};
const std::filesystem::path TestsWithPointCloudIO::kTestFolderPath{
  std::filesystem::temp_directory_path() / "test"};

}  // namespace r2k_replay_test

#endif  // ROS2_KITTI_REPLAY_TEST__TESTS_WITH_POINT_CLOUD_IO_HPP_
