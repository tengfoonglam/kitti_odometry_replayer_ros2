#ifndef ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_
#define ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace r2k_core
{

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudPCLType = pcl::PointCloud<pcl::PointXYZI>;

static constexpr std::size_t kNumberDigitsPCFilename{6};
static constexpr const char kKittiPCExtension[]{".bin"};

/**
 * @brief Load point cloud given a provided path
 *
 * @param pc_bin_path - Path to load point cloud from
 * @return PointCloudMsg::SharedPtr - Loaded point cloud. Empty pointer if failed
 */
[[nodiscard]] PointCloudMsg::SharedPtr load_point_cloud_from_file(
  const std::filesystem::path & pc_bin_path);

/**
 * @brief Check if an point cloud file adheres to KITTI naming convention
 *
 * @param image_path - Path to load poiint cloud from
 * @return true - Is KITTI image file
 * @return false - Otherwise
 */
[[nodiscard]] bool is_kitti_point_cloud_file(const std::filesystem::path & pc_path);

/**
 * @brief Given an index and path, get the full path of the point cloud with the correct KITTI
 * naming convension and file extension
 *
 * @param idx
 * @param folder_path
 * @return std::filesystem::path - Resulting path
 */
[[nodiscard]] std::filesystem::path from_index_to_point_cloud_file_path(
  std::size_t idx, const std::filesystem::path & folder_path);

/**
 * @brief In a folder, find the point cloud file with the largest index
 *
 * @param pc_path - Folder to look for the file
 * @return std::optional<std::size_t> - Largest index, if available
 */
[[nodiscard]] std::optional<std::size_t> get_last_index_of_point_cloud_sequence(
  const std::filesystem::path & pc_path);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POINT_CLOUD_UTILS_HPP_
