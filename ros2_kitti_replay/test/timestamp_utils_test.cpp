#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ros2_kitti_replay/timestamp_utils.hpp>
#include <ros2_kitti_replay_test/test_utils.hpp>
#include <string_view>

class TestExtractTimestampsFromFile : public r2k_replay_test::TestWithIO
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
    EXPECT_DOUBLE_EQ(extracted_timestamps.at(i).seconds(), kTestTimestamps.at(i));
  }
}

TEST_F(TestExtractTimestampsFromFile, EmptyFile)
{
  const auto txt_file_path = kTestFolderPath / "timestamp_file.txt";
  write_timestamps_file(txt_file_path, std::array<decltype(kTestTimestamps)::value_type, 0>{});
  ASSERT_TRUE(std::filesystem::exists(txt_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(txt_file_path);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result.value().empty());
}
