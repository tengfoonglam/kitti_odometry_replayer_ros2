#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <filesystem>
#include <ros2_kitti_replay/point_cloud_utils.hpp>

class TestLoadPointCloudFromFile : public ::testing::Test
{
public:
  using KITTIPoint = std::array<float, 4>;
  using KITTIPoints = std::vector<KITTIPoint>;

  static const std::filesystem::path kTestFolderPath;
  static const KITTIPoints kTestPoints;

  static void write_bin_file(const std::filesystem::path & file_path, const KITTIPoints & points)
  {
    const auto number_points = points.size();
    constexpr auto number_fields = KITTIPoint{}.size();
    std::vector<KITTIPoint::value_type> data;
    data.reserve(number_points * number_fields);
    for (const auto & point : points) {
      data.insert(data.end(), point.cbegin(), point.cend());
    }

    // Example code as provided by the KITTI README
    auto * stream_ptr = fopen(file_path.string().c_str(), "wb");
    if (!points.empty()) {
      auto * data_ptr = reinterpret_cast<const char *>(&data[0]);
      fwrite(data_ptr, sizeof(KITTIPoint::value_type), number_fields * number_points, stream_ptr);
    }
    fclose(stream_ptr);
    ASSERT_TRUE(std::filesystem::exists(file_path));
  }

protected:
  void SetUp() { std::filesystem::create_directory(kTestFolderPath); }
  void TearDown() { std::filesystem::remove_all(kTestFolderPath); }
};
const TestLoadPointCloudFromFile::KITTIPoints TestLoadPointCloudFromFile::kTestPoints{
  {1.0f, 2.0f, 3.0f, 4.0f},
  {5.0f, 6.0f, 7.0f, 8.0f},
  {9.0f, 10.0f, 11.0f, 12.0f},
  {13.0f, 14.0f, 15.0f, 16.0f},
  {17.0f, 18.0f, 19.0f, 20.0f}};
const std::filesystem::path TestLoadPointCloudFromFile::kTestFolderPath{
  std::filesystem::temp_directory_path() / "test"};

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
  for (size_t i = 0; i < kTestPoints.size(); i++) {
    ASSERT_EQ(pcl_cloud[i].x, kTestPoints[i][0]);
    ASSERT_EQ(pcl_cloud[i].y, kTestPoints[i][1]);
    ASSERT_EQ(pcl_cloud[i].z, kTestPoints[i][2]);
    ASSERT_EQ(pcl_cloud[i].intensity, kTestPoints[i][3]);
  }
}
