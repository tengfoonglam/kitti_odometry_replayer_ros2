#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

std::optional<Timestamps> extract_timestamps_from_file(const std::filesystem::path & times_path)
{
  // Check if text file is .txt file and exists
  if (!std::filesystem::exists(times_path) || times_path.extension().string() != ".txt") {
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
    const double timestamp_seconds = std::atof(line.c_str());
    output.emplace_back(static_cast<std::int64_t>(timestamp_seconds * 1e9));
  }

  return output;
}

}  // namespace r2k_core
