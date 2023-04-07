#ifndef ROS2_KITTI_REPLAY__TIMESTAMPS_HPP_
#define ROS2_KITTI_REPLAY__TIMESTAMPS_HPP_

#include <filesystem>
#include <optional>
#include <rclcpp/time.hpp>
#include <vector>

namespace r2k_replay
{
using Timestamp = rclcpp::Time;
using Timestamps = std::vector<rclcpp::Time>;

/**
 * @brief      Extract timestamps from a KITTI times .txt file
 * @note       It is the user's responsibility to ensure that
 *             1) Each line in the file corresponds to a timestamp
 *             2) Timestamps are positive
 *
 * @param[in]  times_path  Path to the times file
 *
 * @return     Success: Extracted timestamps. Failure: std::nullopt
 */
[[nodiscard]] std::optional<Timestamps> extract_timestamps_from_file(
  const std::filesystem::path & times_path);

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__TIMESTAMPS_HPP_
