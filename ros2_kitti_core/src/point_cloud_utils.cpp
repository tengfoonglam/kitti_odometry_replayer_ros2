#include "ros2_kitti_core/point_cloud_utils.hpp"

#include <cstdio>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ros2_kitti_core/data_utils.hpp"

namespace r2k_core
{

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path)
{
  // Check if text file is .bin file and exists
  if (!file_exists_and_correct_extension(pc_bin_path, std::string{kKittiPCExtension})) {
    return PointCloudMsg::SharedPtr();
  }

  // Create and populate Point Cloud Message
  auto output_ptr = std::make_shared<PointCloudMsg>();

  // Read binary file and load it into data field
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
  return is_numbered_file_with_correction_extension(
    pc_path, kNumberDigitsPCFilename, std::string{kKittiPCExtension});
}

[[nodiscard]] std::filesystem::path from_index_to_point_cloud_file_path(
  const std::size_t idx, const std::filesystem::path & folder_path)
{
  return from_index_to_file_path(idx, folder_path, kNumberDigitsPCFilename, kKittiPCExtension);
}

[[nodiscard]] std::optional<std::size_t> get_last_index_of_point_cloud_sequence(
  const std::filesystem::path & pc_path)
{
  return get_last_index_of_data_sequence(pc_path, kNumberDigitsPCFilename, kKittiPCExtension);
}

}  // namespace r2k_core
