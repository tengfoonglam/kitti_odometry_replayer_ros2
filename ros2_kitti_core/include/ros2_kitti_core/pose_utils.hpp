#ifndef ROS2_KITTI_CORE__POSES_HPP_
#define ROS2_KITTI_CORE__TRANSFORMS_HPP_

#include <filesystem>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace r2k_core
{

using Transform = geometry_msgs::msg::Transform;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Transforms = std::vector<Transform>;

/**
 * @brief Given a KITTI pose .txt file, extract all poses in it
 *
 * @param poses_path - Path of .txt file
 * @return std::optional<Transforms> - Extracted poses, std::nullopt if failure
 */
[[nodiscard]] std::optional<Transforms> extract_poses_from_file(
  const std::filesystem::path & poses_path);

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__TRANSFORMS_HPP_
