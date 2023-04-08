#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>

#include <ros2_kitti_replay/pose_data_loader.hpp>
#include <ros2_kitti_replay/timestamp_utils.hpp>
#include <ros2_kitti_replay_test/test_utils.hpp>
#include <ros2_kitti_replay_test/test_with_pose_io.hpp>
#include <vector>

class TestPoseDataLoader : public r2k_replay_test::TestWithPoseIO
{
public:
  using PoseDataLoader = r2k_replay::PoseDataLoader;

  static const tf2::Transform kTestPose;
  static constexpr const char kFrameId[]{"test_base_frame"};
  static constexpr const char kChildFrameFrameId[]{"test_child_frame"};

  [[nodiscard]] static auto get_test_header()
  {
    r2k_replay::PoseDataLoader::Header header;
    header.frame_id = std::string{kFrameId};
    return header;
  }
};
const tf2::Transform TestPoseDataLoader::kTestPose{{0.5, -0.5, 0.5, -0.5}, {4.0, 5.0, 6.0}};

TEST_F(TestPoseDataLoader, ReadFileFailure)
{
  const auto non_existent_file_path = kTestFolderPath / "non_existent_file.txt";
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));

  const auto timestamps = r2k_replay_test::generate_test_timestamps(1, 3);
  ASSERT_GT(timestamps.size(), std::size_t{0});

  r2k_replay::PoseDataLoader loader(
    "test_pose_loader", get_test_header(), std::string{kChildFrameFrameId});

  ASSERT_FALSE(loader.ready());
  ASSERT_FALSE(loader.setup(timestamps, non_existent_file_path));
  ASSERT_FALSE(loader.ready());
}

TEST_F(TestPoseDataLoader, NumberTimestampAndPosesMismatch)
{
  constexpr std::size_t number_timestamps = 10;
  constexpr std::size_t number_poses = 9;
  ASSERT_NE(number_timestamps, number_poses);

  const auto pose_file_path = kTestFolderPath / "pose_file.txt";
  write_poses_file(pose_file_path, std::vector(number_poses, kTestPose));
  ASSERT_TRUE(std::filesystem::exists(pose_file_path));

  const auto timestamps = r2k_replay_test::generate_test_timestamps(1, number_timestamps);
  ASSERT_EQ(timestamps.size(), number_timestamps);

  r2k_replay::PoseDataLoader loader(
    "test_pose_loader", get_test_header(), std::string{kChildFrameFrameId});

  ASSERT_FALSE(loader.ready());
  ASSERT_FALSE(loader.setup(timestamps, pose_file_path));
  ASSERT_FALSE(loader.ready());
}

TEST_F(TestPoseDataLoader, NormalOperations)
{
  constexpr std::size_t number_readings = 10;
  const auto pose_file_path = kTestFolderPath / "pose_file.txt";
  write_poses_file(pose_file_path, std::vector(number_readings, kTestPose));
  ASSERT_TRUE(std::filesystem::exists(pose_file_path));

  const auto timestamps = r2k_replay_test::generate_test_timestamps(1, number_readings);
  ASSERT_EQ(timestamps.size(), number_readings);

  const auto test_header = get_test_header();
  r2k_replay::PoseDataLoader loader(
    "test_pose_loader", test_header, std::string{kChildFrameFrameId});

  ASSERT_FALSE(loader.ready());
  ASSERT_EQ(loader.data_size(), std::size_t{0});
  ASSERT_TRUE(loader.setup(timestamps, pose_file_path));
  ASSERT_TRUE(loader.ready());
  ASSERT_EQ(loader.data_size(), number_readings);

  for (std::size_t i = 0; i < number_readings; i++) {
    for (std::size_t j = 0; j < number_readings; j++) {
      ASSERT_TRUE(loader.prepare_data(j));
    }
    ASSERT_FALSE(loader.prepare_data(number_readings));
    const auto pose_stamped_opt = loader.get_data(i);
    ASSERT_TRUE(pose_stamped_opt.has_value());
    const auto pose_stamped = pose_stamped_opt.value();
    ASSERT_EQ(pose_stamped.header.frame_id, std::string{kFrameId});
    ASSERT_EQ(pose_stamped.child_frame_id, std::string{kChildFrameFrameId});
    const auto header_timestamp = r2k_replay::Timestamp(pose_stamped.header.stamp);
    ASSERT_EQ(timestamps.at(i).nanoseconds(), header_timestamp.nanoseconds());
    tf2::Transform tf2_pose;
    tf2::fromMsg(pose_stamped.transform, tf2_pose);
    EXPECT_EQ(tf2_pose, kTestPose);
  }
}
