#ifndef ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_
#define ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_

#include <filesystem>
#include <optional>
#include <rclcpp/time.hpp>
#include <set>

namespace r2k_replay
{

using OrderedTimestamps = std::set<rclcpp::Time>;

[[nodiscard]] std::optional<OrderedTimestamps> extract_ordered_timestamps_from_file(
  const std::filesystem::path & times_path);

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_
