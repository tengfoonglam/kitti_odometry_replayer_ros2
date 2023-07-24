#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/core.hpp>
#include <ros2_kitti_core/image_utils.hpp>
#include <ros2_kitti_core_test/test_with_image_io.hpp>

class TestLoadImageFromFile : public r2k_core_test::TestWithImageIO
{
};

TEST_F(TestLoadImageFromFile, NonExistentTest)
{
  const auto non_existent_file_path =
    kTestFolderPath / (std::string{"non_existent_file"} + r2k_core::kKittiImageExtension);
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto img_ptr = r2k_core::load_image_from_file(non_existent_file_path);
  ASSERT_FALSE(img_ptr);
}

TEST_F(TestLoadImageFromFile, IncorrectExtensionFileTest)
{
  const auto incorrect_extension_file_path = kTestFolderPath / "incorrect_extension.jpeg";
  write_sample_image(incorrect_extension_file_path, r2k_core::kGrayImageOpenCVType);
  ASSERT_TRUE(std::filesystem::exists(incorrect_extension_file_path));
  const auto img_ptr = r2k_core::load_image_from_file(incorrect_extension_file_path);
  ASSERT_FALSE(img_ptr);
}

TEST_F(TestLoadImageFromFile, IncorrectFormatFileTest)
{
  static constexpr auto kIncorrectFormat{CV_16UC4};
  const auto incorrect_extension_file_path =
    kTestFolderPath / (std::string{"incorrect_format"} + r2k_core::kKittiImageExtension);
  write_sample_image(incorrect_extension_file_path, kIncorrectFormat);
  ASSERT_TRUE(std::filesystem::exists(incorrect_extension_file_path));
  const auto img_ptr = r2k_core::load_image_from_file(incorrect_extension_file_path);
  ASSERT_FALSE(img_ptr);
}

TEST_F(TestLoadImageFromFile, NormalOperation)
{
  for (const auto image_type :
       std::array<int, 2>{r2k_core::kGrayImageOpenCVType, r2k_core::kColourImageOpenCVType}) {
    const auto image_file_path =
      kTestFolderPath / (std::string{"000000"} + r2k_core::kKittiImageExtension);
    write_sample_image(image_file_path, image_type);
    ASSERT_TRUE(std::filesystem::exists(image_file_path));
    const auto img_ptr = r2k_core::load_image_from_file(image_file_path);
    ASSERT_TRUE(img_ptr);
    using HeightType = r2k_core::ImageMsg::_height_type;
    using WidthType = r2k_core::ImageMsg::_width_type;
    ASSERT_EQ(img_ptr->height, static_cast<HeightType>(kSampleImageHeight));
    ASSERT_EQ(img_ptr->width, static_cast<WidthType>(kSampleImageWidth));
    ASSERT_TRUE(std::filesystem::remove(image_file_path));
  }
}
