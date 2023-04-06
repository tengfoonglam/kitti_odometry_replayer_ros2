#ifndef ROS2_KITTI_REPLAY_TEST__TEST_UTILS_HPP_
#define ROS2_KITTI_REPLAY_TEST__TEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_replay/timestamps.hpp>

namespace r2k_replay_test
{
[[nodiscard]] inline auto generate_test_timestamps(const std::size_t start, const std::size_t end)
{
  r2k_replay::Timestamps output;
  for (std::size_t i = start; i <= end; i++) {
    output.emplace_back(i, 0);
  }
  return output;
}

class TestWithIO : public ::testing::Test
{
public:
  static const std::filesystem::path kTestFolderPath;

protected:
  void SetUp() { std::filesystem::create_directory(kTestFolderPath); }
  void TearDown() { std::filesystem::remove_all(kTestFolderPath); }
};
const std::filesystem::path TestWithIO::kTestFolderPath{
  std::filesystem::temp_directory_path() / "test"};

}  // namespace r2k_replay_test

#endif  // ROS2_KITTI_REPLAY_TEST__TEST_UTILS_HPP_
