#ifndef ROS2_KITTI_REPLAY__POSES_HPP_
#define ROS2_KITTI_REPLAY__TRANSFORMS_HPP_

#include <filesystem>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace r2k_replay
{

using Transform = geometry_msgs::msg::Transform;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Transforms = std::vector<Transform>;

[[nodiscard]] std::optional<Transforms> extract_poses_from_file(
  const std::filesystem::path & poses_path);

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__TRANSFORMS_HPP_
