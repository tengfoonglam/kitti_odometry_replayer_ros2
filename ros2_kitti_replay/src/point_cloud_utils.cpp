#include "ros2_kitti_replay/point_cloud_utils.hpp"

#include <iostream>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace r2k_replay
{

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path)
{
  // Check if text file is .bin file and exists
  if (
    !std::filesystem::exists(pc_bin_path.parent_path()) ||
    pc_bin_path.extension().string() != ".bin") {
    return PointCloudMsg::SharedPtr();
  }

  // Point cloud extraction code (adapted from KITTI Odometry dataset README)

  // Establish some constants
  constexpr std::size_t expected_max_number_points = 250000;
  constexpr std::size_t number_fields = 4;  // x, y, z, i
  constexpr std::size_t expected_number_floats = expected_max_number_points * number_fields;
  constexpr auto size_float = sizeof(float);

  // Reserve data and establish pointers
  float * data_ptr = reinterpret_cast<float *>(malloc(expected_number_floats * size_float));
  const float * x_ptr = data_ptr + 0;
  const float * y_ptr = data_ptr + 1;
  const float * z_ptr = data_ptr + 2;
  const float * i_ptr = data_ptr + 3;

  // Load point cloud from file
  auto * stream_ptr = fopen(pc_bin_path.string().c_str(), "rb");
  const auto num_points =
    fread(data_ptr, size_float, expected_number_floats, stream_ptr) / number_fields;

  // Create point cloud message
  auto output_ptr = std::make_shared<PointCloudMsg>();
  sensor_msgs::PointCloud2Modifier modifier(*output_ptr);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  modifier.resize(num_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*output_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*output_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*output_ptr, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(*output_ptr, "intensity");

  // Assign data into message
  for (std::size_t i = 0; i < num_points; i++) {
    *iter_x = *x_ptr;
    *iter_y = *y_ptr;
    *iter_z = *z_ptr;
    *iter_i = *i_ptr;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;

    x_ptr += number_fields;
    y_ptr += number_fields;
    z_ptr += number_fields;
    i_ptr += number_fields;
  }

  // Cleanup
  fclose(stream_ptr);
  free(data_ptr);

  return output_ptr;
}

}  // namespace r2k_replay
