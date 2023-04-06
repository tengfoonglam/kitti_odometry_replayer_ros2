#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <filesystem>
#include <ros2_kitti_replay/point_cloud_utils.hpp>
#include <ros2_kitti_replay_test/test_with_point_cloud_io.hpp>

class TestLoadPointCloudFromFile : public r2k_replay_test::TestWithPointCloudIO
{
};

TEST_F(TestLoadPointCloudFromFile, NonExistentTest)
{
  const auto non_existent_file_path =
    kTestFolderPath / (std::string{"non_existent_file"} + r2k_replay::kKittiPCExtention);
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto pc_ptr = r2k_replay::load_point_cloud_from_file(non_existent_file_path);
  ASSERT_FALSE(pc_ptr);
}

TEST_F(TestLoadPointCloudFromFile, NotBinFileTest)
{
  const auto non_bin_file_path = kTestFolderPath / "non_existent_file.pcd";
  write_bin_file(non_bin_file_path, kTestPoints);
  ASSERT_TRUE(std::filesystem::exists(non_bin_file_path));
  const auto pc_ptr = r2k_replay::load_point_cloud_from_file(non_bin_file_path);
  ASSERT_FALSE(pc_ptr);
}

TEST_F(TestLoadPointCloudFromFile, EmptyFile)
{
  const auto empty_file_path =
    kTestFolderPath / (std::string{"empty_file"} + r2k_replay::kKittiPCExtention);
  write_bin_file(empty_file_path, {});
  ASSERT_TRUE(std::filesystem::exists(empty_file_path));
  const auto pc_ptr = r2k_replay::load_point_cloud_from_file(empty_file_path);
  ASSERT_TRUE(pc_ptr);
  r2k_replay::PointCloudPCLType pcl_cloud;
  pcl::fromROSMsg(*pc_ptr, pcl_cloud);
  ASSERT_EQ(pcl_cloud.size(), std::size_t{0});
}

TEST_F(TestLoadPointCloudFromFile, NormalOperation)
{
  const auto bin_file_path =
    kTestFolderPath / (std::string{"000000"} + r2k_replay::kKittiPCExtention);
  write_bin_file(bin_file_path, kTestPoints);
  ASSERT_TRUE(std::filesystem::exists(bin_file_path));
  const auto pc_ptr = r2k_replay::load_point_cloud_from_file(bin_file_path);
  ASSERT_TRUE(pc_ptr);
  r2k_replay::PointCloudPCLType pcl_cloud;
  pcl::fromROSMsg(*pc_ptr, pcl_cloud);
  ASSERT_EQ(pcl_cloud.size(), kTestPoints.size());
  for (std::size_t i = 0; i < kTestPoints.size(); i++) {
    ASSERT_EQ(pcl_cloud[i].x, kTestPoints[i][0]);
    ASSERT_EQ(pcl_cloud[i].y, kTestPoints[i][1]);
    ASSERT_EQ(pcl_cloud[i].z, kTestPoints[i][2]);
    ASSERT_EQ(pcl_cloud[i].intensity, kTestPoints[i][3]);
  }
}

class IsKittiPointCloudFileTest : public ::testing::TestWithParam<std::tuple<std::string, bool>>
{
};

TEST_P(IsKittiPointCloudFileTest, NormalOperation)
{
  const auto [input, expected_answer] = GetParam();
  ASSERT_EQ(r2k_replay::is_kitti_point_cloud_file(std::filesystem::path(input)), expected_answer);
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudUtilsTests, IsKittiPointCloudFileTest,
  ::testing::Values(
    std::make_tuple("", false), std::make_tuple("000000.pcd", false),
    std::make_tuple("12345.bin", false), std::make_tuple("abc000000.bin", false),
    std::make_tuple("000000", false), std::make_tuple("abcdef.bin", false),
    std::make_tuple("000000.bin", true), std::make_tuple("123456.bin", true),
    std::make_tuple("999999.bin", true)));

class FromIndexToPointCloudFilePath
: public ::testing::TestWithParam<std::tuple<std::size_t, std::string, std::string>>
{
};

TEST_P(FromIndexToPointCloudFilePath, NormalOperation)
{
  const auto [idx, pc_path, expected_answer] = GetParam();
  ASSERT_EQ(
    r2k_replay::from_index_to_point_cloud_file_path(idx, pc_path),
    std::filesystem::path(expected_answer));
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudUtilsTests, FromIndexToPointCloudFilePath,
  ::testing::Values(
    std::make_tuple(0, "", "000000.bin"), std::make_tuple(0, "/home/user", "/home/user/000000.bin"),
    std::make_tuple(123, "/home/user", "/home/user/000123.bin"),
    std::make_tuple(123456, "/home/user", "/home/user/123456.bin"),
    std::make_tuple(1234567, "/home/user", "/home/user/1234567.bin")));

class TestGetLastIndexOfPointCLoudSequence
: public r2k_replay_test::TestWithPointCloudIO,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::size_t>, std::optional<std::size_t>>>
{
};

TEST_P(TestGetLastIndexOfPointCLoudSequence, NormalOperation)
{
  const auto [indices, expected_answer] = GetParam();
  write_kitti_bin_files(
    indices, kTestFolderPath, std::vector<KITTIPoints>(indices.size(), KITTIPoints{}));
  const auto output = r2k_replay::get_last_index_of_point_cloud_sequence(kTestFolderPath);
  ASSERT_EQ(output, expected_answer);
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudUtilsTests, TestGetLastIndexOfPointCLoudSequence,
  ::testing::Values(
    std::make_tuple(std::vector<std::size_t>{}, std::nullopt),
    std::make_tuple(std::vector<std::size_t>{1, 2, 3}, std::nullopt),
    std::make_tuple(std::vector<std::size_t>{0}, 0),
    std::make_tuple(std::vector<std::size_t>{0, 1, 2, 3}, 3),
    std::make_tuple(std::vector<std::size_t>{0, 2, 3, 4}, 0),
    std::make_tuple(std::vector<std::size_t>{0, 1, 2, 3, 4}, 4),
    std::make_tuple(std::vector<std::size_t>{4, 3, 2, 1, 0}, 4)));
