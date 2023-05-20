#include "ros2_kitti_core/point_cloud_utils.hpp"

#include <cstdio>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace r2k_core
{

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path)
{
  // Check if text file is .bin file and exists
  if (
    !std::filesystem::exists(pc_bin_path) ||
    pc_bin_path.extension().string() != std::string{kKittiPCExtention}) {
    return PointCloudMsg::SharedPtr();
  }

  // Create and populate Point Cloud Message
  auto output_ptr = std::make_shared<PointCloudMsg>();

  // Read binary file and load it into data field
  // Adapted from readme of KITTI Odometry devkit

  const size_t data_length = std::filesystem::file_size(pc_bin_path);
  output_ptr->data.resize(data_length);
  auto * stream_ptr = std::fopen(pc_bin_path.c_str(), "rb");
  if (!stream_ptr) {
    return PointCloudMsg::SharedPtr();
  }
  constexpr auto element_size = sizeof(decltype(output_ptr->data)::value_type);
  std::fread(output_ptr->data.data(), element_size, data_length, stream_ptr);
  std::fclose(stream_ptr);

  // Compute the number of points
  constexpr std::size_t number_fields = 4;
  constexpr auto size_float = sizeof(float);
  const auto num_points = std::size_t{output_ptr->data.size() / (number_fields * size_float)};

  // Set point cloud details
  output_ptr->height = num_points;
  output_ptr->width = 1;
  output_ptr->is_dense = false;
  output_ptr->is_bigendian = false;

  // Specify fields in the point cloud. Note: This is only done when height and width of the point
  // cloud has been specified as the modifier will automatically resize the message accordingly
  sensor_msgs::PointCloud2Modifier modifier(*output_ptr);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  return output_ptr;
}

[[nodiscard]] bool is_kitti_point_cloud_file(const std::filesystem::path & pc_path)
{
  const bool extension_match = pc_path.extension().string() == std::string{kKittiPCExtention};
  const auto & stem = pc_path.stem().string();
  const bool number_char_match = stem.size() == kNumberDigitsPCFilename;
  const bool stem_all_digits = std::all_of(stem.cbegin(), stem.cend(), ::isdigit);
  return extension_match && number_char_match && stem_all_digits;
}

[[nodiscard]] std::filesystem::path from_index_to_point_cloud_file_path(
  const std::size_t idx, const std::filesystem::path & folder_path)
{
  const auto idx_unpadded = std::to_string(idx);
  const auto number_digits_to_pad =
    kNumberDigitsPCFilename - std::min(kNumberDigitsPCFilename, idx_unpadded.length());
  auto idx_padded = std::string(number_digits_to_pad, '0') + idx_unpadded;
  return folder_path / (idx_padded + std::string{kKittiPCExtention});
}

[[nodiscard]] std::optional<std::size_t> get_last_index_of_point_cloud_sequence(
  const std::filesystem::path & pc_path)
{
  const auto it = std::filesystem::directory_iterator(pc_path);
  const auto number_pc_files = static_cast<std::size_t>(
    std::count_if(std::filesystem::begin(it), std::filesystem::end(it), [](const auto & dir_entry) {
      return dir_entry.is_regular_file() && is_kitti_point_cloud_file(dir_entry.path());
    }));

  if (number_pc_files == 0) {
    return std::nullopt;
  }

  std::optional<std::size_t> answer;
  for (std::size_t idx = 0; idx < number_pc_files; idx++) {
    const auto path_to_check = from_index_to_point_cloud_file_path(idx, pc_path);

    if (std::filesystem::exists(path_to_check)) {
      answer = idx;
    } else {
      break;
    }
  }

  return answer;
}

}  // namespace r2k_core
