#ifndef ROS2_KITTI_REPLAY_TEST__TEST_WITH_IO_HPP_
#define ROS2_KITTI_REPLAY_TEST__TEST_WITH_IO_HPP_

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace r2k_replay_test
{

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

#endif  // ROS2_KITTI_REPLAY_TEST__TEST_WITH_IO_HPP_
