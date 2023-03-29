#ifndef ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_
#define ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace r2k_replay
{

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudPCLType = pcl::PointCloud<pcl::PointXYZI>;

[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path);

[[nodiscard]] bool is_kitti_point_cloud_file(const std::filesystem::path & pc_file);

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__POINT_CLOUD_UTILS_HPP_
