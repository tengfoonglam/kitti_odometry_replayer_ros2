#ifndef ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_
#define ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_

#include <rclcpp/time.hpp>
#include <set>

namespace r2k_replay
{

using OrderedTimestamps = std::set<rclcpp::Time>;

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY_INCLUDE_ROS2_KITTI_REPLAY_ORDERED_TIMESTAMPS_HPP_
