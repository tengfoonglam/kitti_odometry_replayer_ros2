#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros2_kitti_replay/point_cloud_data_loader.hpp>
#include <ros2_kitti_replay/timestamp_utils.hpp>
#include <ros2_kitti_replay_test/test_utils.hpp>
#include <ros2_kitti_replay_test/test_with_point_cloud_io.hpp>

class TestPointCloudDataLoader
: public r2k_replay_test::TestWithPointCloudIO,
  public ::testing::WithParamInterface<std::tuple<std::size_t, std::size_t>>
{
};

TEST_P(TestPointCloudDataLoader, NormalOperation)
{
  const auto [number_timestamps, number_point_clouds] = GetParam();

  const auto timestamps = (number_timestamps == 0)
                            ? r2k_replay::Timestamps{}
                            : r2k_replay_test::generate_test_timestamps(
                                0, number_timestamps > 1 ? number_timestamps - 1 : 0);
  ASSERT_EQ(number_timestamps, timestamps.size());

  std::vector<std::size_t> pc_indices(number_point_clouds);
  std::iota(pc_indices.begin(), pc_indices.end(), 0);
  ASSERT_EQ(number_point_clouds, pc_indices.size());

  std::vector<KITTIPoints> point_clouds(number_point_clouds);
  std::transform(
    pc_indices.cbegin(), pc_indices.cend(), point_clouds.begin(), [](const auto indice) {
      return KITTIPoints(indice, KITTIPoint{1.0f, 2.0f, 3.0f, 4.0f});
    });
  ASSERT_EQ(number_point_clouds, point_clouds.size());

  write_kitti_bin_files(pc_indices, kTestFolderPath, point_clouds);

  r2k_replay::PointCloudDataLoader::Header header;
  header.frame_id = "test_id";

  r2k_replay::PointCloudDataLoader loader{"TestPointCloudDataLoader", header};

  ASSERT_FALSE(loader.prepare_data(0));
  ASSERT_FALSE(loader.get_data(0).has_value());
  ASSERT_FALSE(loader.ready());
  ASSERT_EQ(loader.data_size(), size_t{0});

  if (number_timestamps == 0 || number_point_clouds == 0) {
    ASSERT_FALSE(loader.setup(timestamps, kTestFolderPath));
    ASSERT_FALSE(loader.ready());
    ASSERT_EQ(loader.data_size(), size_t{0});
    return;
  }

  ASSERT_TRUE(loader.setup(timestamps, kTestFolderPath));
  ASSERT_TRUE(loader.ready());
  ASSERT_EQ(loader.data_size(), std::min(number_timestamps, number_point_clouds));

  const auto max_number = std::max(number_timestamps, number_point_clouds);
  for (std::size_t i = 0; i < max_number; i++) {
    if (i < loader.data_size()) {
      ASSERT_TRUE(loader.prepare_data(i));
      const auto data_opt = loader.get_data(i);
      ASSERT_TRUE(data_opt.has_value());

      if (i > 0) {
        ASSERT_FALSE(loader.get_data(i - 1).has_value());
      }
      if (i + 1 < max_number) {
        ASSERT_FALSE(loader.get_data(i + 1).has_value());
      }

      const auto data_ptr = data_opt.value();
      ASSERT_TRUE(static_cast<bool>(data_ptr));
      ASSERT_EQ(header.frame_id, data_ptr->header.frame_id);
      const auto header_timestamp = r2k_replay::Timestamp(data_ptr->header.stamp);
      ASSERT_EQ(timestamps.at(i).nanoseconds(), header_timestamp.nanoseconds());

      r2k_replay::PointCloudPCLType pcl_cloud;
      pcl::fromROSMsg(*data_ptr, pcl_cloud);
      ASSERT_EQ(pcl_cloud.size(), i);

    } else {
      ASSERT_FALSE(loader.prepare_data(i));
      ASSERT_FALSE(loader.get_data(i).has_value());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudDataLoaderTests, TestPointCloudDataLoader,
  ::testing::Values(
    std::make_tuple(0, 0), std::make_tuple(0, 1), std::make_tuple(1, 0), std::make_tuple(0, 10),
    std::make_tuple(10, 0), std::make_tuple(1, 1), std::make_tuple(10, 10), std::make_tuple(1, 2),
    std::make_tuple(5, 10), std::make_tuple(2, 1), std::make_tuple(10, 5)));
