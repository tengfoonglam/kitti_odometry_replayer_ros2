#ifndef ROS2_KITTI_CORE__DATA_UTILS_HPP_
#define ROS2_KITTI_CORE__DATA_UTILS_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace r2k_core
{

/**
 * @brief Given a path, check that it exists and the extension mathes the one provided
 *
 * @param path
 * @param extension
 * @return true - File matches requirements
 * @return false - Otherwise
 */
[[nodiscard]] bool file_exists_and_correct_extension(
  const std::filesystem::path & path, const std::string & extension);

/**
 * @brief Check that given a path the filename (i.e. the stem) has only numbers and is the length of
 * stem_digits and has the correct extension
 *
 * @param path
 * @param stem_digits
 * @param extension
 * @return true - File matches requirements
 * @return false - Otherwise
 */
[[nodiscard]] bool is_numbered_file_with_correction_extension(
  const std::filesystem::path & path, std::size_t stem_digits, const std::string & extension);

/**
 * @brief Convert an index to a file path
 *
 * @param idx - Index
 * @param folder_path - Folder of the path
 * @param stem_digits - Number of digits in the stem
 * @param extension - Extension of file
 * @return std::filesystem::path - Path
 */
[[nodiscard]] std::filesystem::path from_index_to_file_path(
  std::size_t idx, const std::filesystem::path & folder_path, std::size_t stem_digits,
  const std::string & extension);

/**
 * @brief Look through a folder and find the file where the index indicated bu the stem is the
 * largest
 *
 * @param path
 * @param stem_digits
 * @param extension
 * @return std::optional<std::size_t> - Last file, if it exists
 */
[[nodiscard]] std::optional<std::size_t> get_last_index_of_data_sequence(
  const std::filesystem::path & path, std::size_t stem_digits, const std::string & extension);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__DATA_UTILS_HPP_
