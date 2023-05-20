#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_core/pose_utils.hpp>
#include <ros2_kitti_core_test/test_with_pose_io.hpp>

class TestExtractPosesFromFile : public r2k_core_test::TestWithPoseIO
{
public:
  static const std::array<tf2::Transform, 5> kTestPoses;
};
const std::array<tf2::Transform, 5> TestExtractPosesFromFile::kTestPoses{
  {tf2::Transform{{0.0, 0.0, 0.0, 1.0}, {1.0, 2.0, 3.0}},
   tf2::Transform{{0.5, -0.5, 0.5, -0.5}, {4.0, 5.0, 6.0}},
   tf2::Transform{{0.5, -0.5, 0.5, 0.5}, {7.0, 8.0, 9.0}},
   tf2::Transform{{0.5, -0.5, -0.5, -0.5}, {10.0, 11.0, 12.0}},
   tf2::Transform{{0.5, -0.5, -0.5, 0.5}, {13.0, 14.0, 15.0}}}};

TEST_F(TestExtractPosesFromFile, NonExistentTest)
{
  const auto non_existent_file_path = kTestFolderPath / "non_existent_file.txt";
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto result = r2k_core::extract_poses_from_file(non_existent_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractPosesFromFile, NotTxtFileTest)
{
  const auto non_txt_file_path = kTestFolderPath / "timestamp_file.csv";
  write_poses_file(non_txt_file_path, kTestPoses);
  ASSERT_TRUE(std::filesystem::exists(non_txt_file_path));
  const auto result = r2k_core::extract_poses_from_file(non_txt_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractPosesFromFile, NormalOperation)
{
  const auto txt_file_path = kTestFolderPath / "pose_file.txt";
  write_poses_file(txt_file_path, kTestPoses);
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_core::extract_poses_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  const auto extracted_poses = result.value();
  ASSERT_EQ(extracted_poses.size(), extracted_poses.size());
  for (std::size_t i = 0; i < std::min(extracted_poses.size(), kTestPoses.size()); i++) {
    tf2::Transform tf2_pose;
    tf2::fromMsg(extracted_poses.at(i), tf2_pose);
    EXPECT_EQ(tf2_pose, kTestPoses.at(i));
  }
}

TEST_F(TestExtractPosesFromFile, EmptyFile)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_poses_file(txt_file_path, std::array<decltype(kTestPoses)::value_type, 0>{});
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_core::extract_poses_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result.value().empty());
}
