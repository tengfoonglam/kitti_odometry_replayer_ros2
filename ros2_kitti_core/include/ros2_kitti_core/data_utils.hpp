#ifndef ROS2_KITTI_CORE__DATA_UTILS_HPP_
#define ROS2_KITTI_CORE__DATA_UTILS_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace r2k_core
{

[[nodiscard]] bool file_exists_and_correct_extension(
  const std::filesystem::path & path, const std::string & extension);

[[nodiscard]] bool is_numbered_file_with_correction_extension(
  const std::filesystem::path & path, const std::size_t stem_digits, const std::string & extension);

[[nodiscard]] std::filesystem::path from_index_to_file_path(
  const std::size_t idx, const std::filesystem::path & folder_path, const std::size_t stem_digits,
  const std::string & extension);

[[nodiscard]] std::optional<std::size_t> get_last_index_of_data_sequence(
  const std::filesystem::path & path, const std::size_t stem_digits, const std::string & extension);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__DATA_UTILS_HPP_
