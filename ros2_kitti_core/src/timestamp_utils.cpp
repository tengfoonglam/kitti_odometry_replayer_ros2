#include "ros2_kitti_core/timestamp_utils.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "ros2_kitti_core/data_utils.hpp"

namespace r2k_core
{

std::optional<Timestamps> extract_timestamps_from_file(const std::filesystem::path & times_path)
{
  // Check if text file is .txt file and exists
  if (!file_exists_and_correct_extension(times_path, ".txt")) {
    return std::nullopt;
  }

  // Open file. Note: Upon destruction, open file is automatically closed
  std::ifstream times_file;
  times_file.open(times_path, std::ios::in);

  if (!times_file.is_open()) {
    return std::nullopt;
  }

  // If successful, parse each line as a double then convert to ros time
  Timestamps output;
  for (std::string line; std::getline(times_file, line);) {
    output.push_back(to_timestamp(std::stod(line)));
  }

  return output;
}

}  // namespace r2k_core
