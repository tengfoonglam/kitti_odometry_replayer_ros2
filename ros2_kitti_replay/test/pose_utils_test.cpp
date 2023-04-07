#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_replay/pose_utils.hpp>
#include <ros2_kitti_replay_test/test_utils.hpp>

class TestExtractPosesFromFile : public r2k_replay_test::TestWithIO
{
public:
  static const std::array<tf2::Transform, 5> kTestPoses;

  template <typename T>
  static void write_poses_file(const std::filesystem::path & file_path, const T & poses)
  {
    std::ofstream output_file_stream;
    output_file_stream.open(file_path, std::ios::out);
    for (const auto & pose : poses) {
      const auto rotation_matrix = pose.getBasis();
      const auto position = pose.getOrigin();
      std::array file_row{rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
                          position[0],           rotation_matrix[1][0], rotation_matrix[1][1],
                          rotation_matrix[1][2], position[1],           rotation_matrix[2][0],
                          rotation_matrix[2][1], rotation_matrix[2][2], position[2]};

      for (const auto element : file_row) {
        output_file_stream << std::scientific << element;
        const auto last_element = (&element == &file_row.back());
        if (!last_element) {
          output_file_stream << ' ';
        }
      }

      const auto last_pose = (&pose == &poses.back());
      if (!last_pose) {
        output_file_stream << '\n';
      }
    }
    output_file_stream.close();
  }
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
  const auto result = r2k_replay::extract_poses_from_file(non_existent_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractPosesFromFile, NotTxtFileTest)
{
  const auto non_txt_file_path = kTestFolderPath / "timestamp_file.csv";
  write_poses_file(non_txt_file_path, kTestPoses);
  ASSERT_TRUE(std::filesystem::exists(non_txt_file_path));
  const auto result = r2k_replay::extract_poses_from_file(non_txt_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractPosesFromFile, NormalOperation)
{
  const auto txt_file_path = kTestFolderPath / "pose_file.txt";
  write_poses_file(txt_file_path, kTestPoses);
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_replay::extract_poses_from_file(txt_file_path);
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
  const auto result = r2k_replay::extract_poses_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result.value().empty());
}
