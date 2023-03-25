#ifndef ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_
#define ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_

#include <filesystem>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace r2k_replay
{

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path);

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_
