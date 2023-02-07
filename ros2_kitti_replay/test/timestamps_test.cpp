#include <gtest/gtest.h>

#include <filesystem>
#include <ros2_kitti_replay/timestamps.hpp>

// Non-existent file
TEST(TestExtractTimestampsFromFile, NonExistentTest)
{
  const auto non_existent_file_path =
    std::filesystem::temp_directory_path() / "non_existent_file.txt";
  ASSERT_FALSE(std::filesystem::exists(non_existent_file_path));
  const auto result = r2k_replay::extract_timestamps_from_file(non_existent_file_path);
  ASSERT_FALSE(result.has_value());
}

// Non txt file

// Normal operations

// Empty file
