#include "ros2_kitti_core/data_utils.hpp"

#include <algorithm>

namespace r2k_core
{

bool file_exists_and_correct_extension(
  const std::filesystem::path & path, const std::string & extension)
{
  return (std::filesystem::exists(path) && path.extension().string() == std::string{extension});
}

bool is_numbered_file_with_correction_extension(
  const std::filesystem::path & path, const std::size_t stem_digits, const std::string & extension)
{
  const bool extension_match = path.extension().string() == extension;
  const auto & stem = path.stem().string();
  const bool number_char_match = stem.size() == stem_digits;
  const bool stem_all_digits = std::all_of(stem.cbegin(), stem.cend(), ::isdigit);
  return extension_match && number_char_match && stem_all_digits;
}

std::filesystem::path from_index_to_file_path(
  const std::size_t idx, const std::filesystem::path & folder_path, const std::size_t stem_digits,
  const std::string & extension)
{
  const auto idx_unpadded = std::to_string(idx);
  const auto number_digits_to_pad = stem_digits - std::min(stem_digits, idx_unpadded.length());
  auto idx_padded = std::string(number_digits_to_pad, '0') + idx_unpadded;
  return folder_path / (idx_padded + extension);
}

std::optional<std::size_t> get_last_index_of_data_sequence(
  const std::filesystem::path & path, const std::size_t stem_digits, const std::string & extension)
{
  const auto it = std::filesystem::directory_iterator(path);
  const auto number_pc_files = static_cast<std::size_t>(std::count_if(
    std::filesystem::begin(it), std::filesystem::end(it),
    [stem_digits, extension](const auto & dir_entry) {
      return dir_entry.is_regular_file() &&
             is_numbered_file_with_correction_extension(dir_entry.path(), stem_digits, extension);
    }));

  if (number_pc_files == 0) {
    return std::nullopt;
  }

  std::optional<std::size_t> answer;
  for (std::size_t idx = 0; idx < number_pc_files; idx++) {
    const auto path_to_check = from_index_to_file_path(idx, path, stem_digits, extension);

    if (std::filesystem::exists(path_to_check)) {
      answer = idx;
    } else {
      break;
    }
  }

  return answer;
}

}  // namespace r2k_core
