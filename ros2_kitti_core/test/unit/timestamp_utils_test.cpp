#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <ros2_kitti_core_test/test_with_io.hpp>
#include <string_view>

class TestToTimestamp : public ::testing::TestWithParam<double>
{
};

TEST_P(TestToTimestamp, NormalOperation)
{
  const auto seconds = GetParam();
  ASSERT_DOUBLE_EQ(r2k_core::to_timestamp(seconds).seconds(), seconds);
}

INSTANTIATE_TEST_SUITE_P(
  TestTimestampUtils, TestToTimestamp,
  ::testing::Values(
    -1234567890.1234567890l, -1.23456l, -5.0l, -1.0l, 0.0l, 1.0l, 5.0l, 1.23456l,
    1234567890.1234567890l));

class TestExtractTimestampsFromFile : public r2k_core_test::TestWithIO
{
public:
  static constexpr std::array<double, 3> kTestTimestamps{1.0, 2.0, 3.0};

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
};

TEST_F(TestExtractTimestampsFromFile, NonExistentTest)
{
  const auto non_existent_file_path = kTestFolderPath / "non_existent_file.txt";
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto result = r2k_core::extract_timestamps_from_file(non_existent_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractTimestampsFromFile, NotTxtFileTest)
{
  const auto non_txt_file_path = kTestFolderPath / "timestamp_file.csv";
  write_timestamps_file(non_txt_file_path, kTestTimestamps);
  ASSERT_TRUE(std::filesystem::exists(non_txt_file_path));
  const auto result = r2k_core::extract_timestamps_from_file(non_txt_file_path);
  ASSERT_FALSE(result.has_value());
}

TEST_F(TestExtractTimestampsFromFile, NormalOperation)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_timestamps_file(txt_file_path, kTestTimestamps);
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_core::extract_timestamps_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  const auto extracted_timestamps = result.value();
  ASSERT_EQ(extracted_timestamps.size(), kTestTimestamps.size());
  for (std::size_t i = 0; i < std::min(extracted_timestamps.size(), kTestTimestamps.size()); i++) {
    EXPECT_DOUBLE_EQ(extracted_timestamps.at(i).seconds(), kTestTimestamps.at(i));
  }
}

TEST_F(TestExtractTimestampsFromFile, EmptyFile)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_timestamps_file(txt_file_path, std::array<decltype(kTestTimestamps)::value_type, 0>{});
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_core::extract_timestamps_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result.value().empty());
}
