#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_replay/timestamps.hpp>
#include <string_view>

class TestExtractTimestampsFromFile : public ::testing::Test
{
public:
  static constexpr std::array<double, 3> kTestTimestamps{1.0, 2.0, 3.0};
  static const std::filesystem::path kTestFolderPath;

  template <typename T>
  static void write_timestamps_file(const std::filesystem::path & file_path, const T & timestamps)
  {
    std::ofstream output_file_stream;
    output_file_stream.open(file_path, std::ios::out);
    for (const auto & timestamp : timestamps) {
      output_file_stream << std::scientific << timestamp;

      const auto last_element = (&timestamp == &timestamps.back());
      if (!last_element) {
        output_file_stream << '\n';
      }
    }
    output_file_stream.close();
  }

protected:
  void SetUp() { std::filesystem::create_directory(kTestFolderPath); }
  void TearDown() { std::filesystem::remove_all(kTestFolderPath); }
};
const std::filesystem::path TestExtractTimestampsFromFile::kTestFolderPath{
  std::filesystem::temp_directory_path() / "test"};

TEST_F(TestExtractTimestampsFromFile, NonExistentTest)
{
  const auto non_existent_file_path = kTestFolderPath / "non_existent_file.txt";
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(non_existent_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractTimestampsFromFile, NotTxtFileTest)
{
  const auto non_txt_file_path = kTestFolderPath / "timestamp_file.csv";
  write_timestamps_file(non_txt_file_path, kTestTimestamps);
  ASSERT_TRUE(std::filesystem::exists(non_txt_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(non_txt_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractTimestampsFromFile, NormalOperation)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_timestamps_file(txt_file_path, kTestTimestamps);
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  const auto extracted_timestamps = result.value();
  ASSERT_EQ(extracted_timestamps.size(), kTestTimestamps.size());
  for (std::size_t i = 0; i < std::min(extracted_timestamps.size(), kTestTimestamps.size()); i++) {
    EXPECT_DOUBLE_EQ(extracted_timestamps[i].seconds(), kTestTimestamps[i]);
  }
}

TEST_F(TestExtractTimestampsFromFile, EmptyFile)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_timestamps_file(txt_file_path, std::vector<double>{});
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  const auto extracted_timestamps = result.value();
  ASSERT_TRUE(extracted_timestamps.empty());
}
