#include <gtest/gtest.h>

#include <ros2_kitti_core/image_data_loader.hpp>
#include <ros2_kitti_core/image_utils.hpp>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <ros2_kitti_core_test/test_utils.hpp>
#include <ros2_kitti_core_test/test_with_image_io.hpp>

class TestImageDataLoader
: public r2k_core_test::TestWithImageIO,
  public ::testing::WithParamInterface<std::tuple<std::size_t, std::size_t>>
{
};

TEST_P(TestImageDataLoader, NormalOperation)
{
  const auto [number_timestamps, number_images] = GetParam();

  const auto timestamps = (number_timestamps == 0)
                            ? r2k_core::Timestamps{}
                            : r2k_core_test::generate_test_timestamps(
                                0, number_timestamps > 1 ? number_timestamps - 1 : 0);
  ASSERT_EQ(number_timestamps, timestamps.size());

  std::vector<std::size_t> image_indices(number_images);
  std::iota(std::begin(image_indices), std::end(image_indices), 0);
  ASSERT_EQ(number_images, image_indices.size());

  write_example_image_files(image_indices, kTestFolderPath, r2k_core::kGrayImageOpenCVType);

  r2k_core::ImageDataLoader::Header header;
  header.frame_id = "test_id";

  r2k_core::ImageDataLoader loader{"TestImageDataLoader", header};

  ASSERT_FALSE(loader.prepare_data(0));
  ASSERT_FALSE(loader.get_data(0).has_value());
  ASSERT_FALSE(loader.ready());
  ASSERT_EQ(loader.data_size(), size_t{0});

  if (number_timestamps == 0 || number_images == 0) {
    ASSERT_FALSE(loader.setup(timestamps, kTestFolderPath));
    ASSERT_FALSE(loader.ready());
    ASSERT_EQ(loader.data_size(), size_t{0});
    return;
  }

  ASSERT_TRUE(loader.setup(timestamps, kTestFolderPath));
  ASSERT_TRUE(loader.ready());
  ASSERT_EQ(loader.data_size(), std::min(number_timestamps, number_images));

  const auto max_number = std::max(number_timestamps, number_images);
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
      const auto header_timestamp = r2k_core::Timestamp(data_ptr->header.stamp);
      ASSERT_EQ(timestamps.at(i).nanoseconds(), header_timestamp.nanoseconds());

      using HeightType = r2k_core::ImageMsg::_height_type;
      using WidthType = r2k_core::ImageMsg::_width_type;
      ASSERT_EQ(data_ptr->height, static_cast<HeightType>(kSampleImageHeight));
      ASSERT_EQ(data_ptr->width, static_cast<WidthType>(kSampleImageWidth));

    } else {
      ASSERT_FALSE(loader.prepare_data(i));
      ASSERT_FALSE(loader.get_data(i).has_value());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  ImageDataLoaderTests, TestImageDataLoader,
  ::testing::Values(
    std::make_tuple(0, 0), std::make_tuple(0, 1), std::make_tuple(1, 0), std::make_tuple(0, 10),
    std::make_tuple(10, 0), std::make_tuple(1, 1), std::make_tuple(10, 10), std::make_tuple(1, 2),
    std::make_tuple(5, 10), std::make_tuple(2, 1), std::make_tuple(10, 5)));
