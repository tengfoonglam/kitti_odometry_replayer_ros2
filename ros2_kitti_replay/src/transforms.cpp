#include "ros2_kitti_replay/transforms.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <string>

namespace r2k_replay
{

[[nodiscard]] std::optional<Transforms> extract_poses_from_file(
  const std::filesystem::path & poses_path)
{
  // Check if text file is .txt file and exists
  if (!std::filesystem::exists(poses_path) || poses_path.extension().string() != ".txt") {
    return std::nullopt;
  }

  // Open file. Note: Upon destruction, open file is automatically closed
  std::ifstream times_file;
  times_file.open(poses_path, std::ios::in);

  if (!times_file.is_open()) {
    return std::nullopt;
  }

  // If successful, parse each line
  Transforms output;
  constexpr std::size_t expected_number_elements = 12;
  constexpr char delimiter = ' ';

  for (std::string line; std::getline(times_file, line);) {
    // Split lines into numbers
    std::stringstream line_stream(line);
    std::string item;
    std::vector<std::string> string_elements;
    string_elements.reserve(expected_number_elements);
    while (std::getline(line_stream, item, delimiter)) {
      string_elements.push_back(item);
    }

    // If correct number of numbers, convert to transform and add to output
    if (string_elements.size() != expected_number_elements) {
      continue;
    }

    std::array<double, expected_number_elements> elems;
    std::transform(
      string_elements.cbegin(), string_elements.cend(), elems.begin(),
      [](const auto & str) -> double { return std::atof(str.c_str()); });

    const tf2::Vector3 translation{elems[3], elems[7], elems[11]};
    const tf2::Matrix3x3 rotation{elems[0], elems[1], elems[2], elems[4], elems[5],
                                  elems[6], elems[8], elems[9], elems[10]};
    tf2::Transform tf2_transform;
    tf2_transform.setOrigin(translation);
    tf2_transform.setBasis(rotation);
    output.push_back(tf2::toMsg(tf2_transform));
  }

  return output;
}

}  // namespace r2k_replay
