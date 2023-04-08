#ifndef ROS2_KITTI_REPLAY_TEST__TEST_WITH_POSE_IO_HPP_
#define ROS2_KITTI_REPLAY_TEST__TEST_WITH_POSE_IO_HPP_

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <ros2_kitti_replay/pose_utils.hpp>
#include <string>
#include <vector>

#include "ros2_kitti_replay_test/test_with_io.hpp"

namespace r2k_replay_test
{

class TestWithPoseIO : public r2k_replay_test::TestWithIO
{
public:
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

}  // namespace r2k_replay_test

#endif  // ROS2_KITTI_REPLAY_TEST__TEST_WITH_POSE_IO_HPP_
