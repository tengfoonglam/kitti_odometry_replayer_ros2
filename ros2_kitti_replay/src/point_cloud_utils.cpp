#include "ros2_kitti_replay/point_cloud_utils.hpp"

#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace r2k_replay
{

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path)
{
  // Check if text file is .bin file and exists
  if (!std::filesystem::exists(pc_bin_path) || pc_bin_path.extension().string() != ".bin") {
    return PointCloudMsg::SharedPtr();
  }

  // Create and populate Point Cloud Message
  auto output_ptr = std::make_shared<PointCloudMsg>();

  // Read binary file and load it into data field
  std::ifstream stream(pc_bin_path.string(), std::ios::in | std::ios::binary);
  output_ptr->data = decltype(output_ptr->data){
    (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>()};

  // Compute the number of points
  constexpr std::size_t number_fields = 4;
  constexpr auto size_float = sizeof(float);
  const auto num_points = output_ptr->data.size() / (number_fields * size_float);

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

}  // namespace r2k_replay
