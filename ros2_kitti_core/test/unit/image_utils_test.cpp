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

class IsKittiImageFileTest : public ::testing::TestWithParam<std::tuple<std::string, bool>>
{
};

TEST_P(IsKittiImageFileTest, NormalOperation)
{
  const auto [input, expected_answer] = GetParam();
  ASSERT_EQ(r2k_core::is_kitti_image_file(std::filesystem::path(input)), expected_answer);
}

INSTANTIATE_TEST_SUITE_P(
  ImageUtilsTests, IsKittiImageFileTest,
  ::testing::Values(
    std::make_tuple("", false), std::make_tuple("000000.jpeg", false),
    std::make_tuple("12345.png", false), std::make_tuple("1234567.png", false),
    std::make_tuple("abc000000.png", false), std::make_tuple("000000", false),
    std::make_tuple("abcdef.png", false), std::make_tuple("000000.png", true),
    std::make_tuple("123456.png", true), std::make_tuple("999999.png", true)));

class TestFromIndexToImageFilePath
: public ::testing::TestWithParam<std::tuple<std::size_t, std::string, std::string>>
{
};

TEST_P(TestFromIndexToImageFilePath, NormalOperation)
{
  const auto [idx, image_path, expected_answer] = GetParam();
  ASSERT_EQ(
    r2k_core::from_index_to_image_file_path(idx, image_path),
    std::filesystem::path(expected_answer));
}

INSTANTIATE_TEST_SUITE_P(
  ImageUtilsTests, TestFromIndexToImageFilePath,
  ::testing::Values(
    std::make_tuple(0, "", "000000.png"), std::make_tuple(0, "/home/user", "/home/user/000000.png"),
    std::make_tuple(123, "/home/user", "/home/user/000123.png"),
    std::make_tuple(123456, "/home/user", "/home/user/123456.png"),
    std::make_tuple(1234567, "/home/user", "/home/user/1234567.png")));

class TestGetLastIndexOfImageSequence
: public r2k_core_test::TestWithImageIO,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::size_t>, std::optional<std::size_t>>>
{
};

TEST_P(TestGetLastIndexOfImageSequence, NormalOperation)
{
  const auto [indices, expected_answer] = GetParam();
  write_example_image_files(indices, kTestFolderPath, r2k_core::kGrayImageOpenCVType);
  const auto output = r2k_core::get_last_index_of_image_sequence(kTestFolderPath);
  ASSERT_EQ(output, expected_answer);
}

INSTANTIATE_TEST_SUITE_P(
  ImageUtilsTests, TestGetLastIndexOfImageSequence,
  ::testing::Values(
    std::make_tuple(std::vector<std::size_t>{}, std::nullopt),
    std::make_tuple(std::vector<std::size_t>{1, 2, 3}, std::nullopt),
    std::make_tuple(std::vector<std::size_t>{0}, 0),
    std::make_tuple(std::vector<std::size_t>{0, 1, 2, 3}, 3),
    std::make_tuple(std::vector<std::size_t>{0, 2, 3, 4}, 0),
    std::make_tuple(std::vector<std::size_t>{0, 1, 2, 3, 4}, 4),
    std::make_tuple(std::vector<std::size_t>{4, 3, 2, 1, 0}, 4)));
