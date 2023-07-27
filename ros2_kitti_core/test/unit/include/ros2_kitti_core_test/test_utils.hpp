#ifndef ROS2_KITTI_CORE_TEST__TEST_UTILS_HPP_
#define ROS2_KITTI_CORE_TEST__TEST_UTILS_HPP_

#include <ros2_kitti_core/timestamp_utils.hpp>

namespace r2k_core_test
{
[[nodiscard]] inline auto generate_test_timestamps(std::size_t start, std::size_t end)
{
  r2k_core::Timestamps output;
  for (std::size_t i = start; i <= end; i++) {
    output.emplace_back(i, 0);
  }
  return output;
}
}  // namespace r2k_core_test

#endif  // ROS2_KITTI_CORE_TEST__TEST_UTILS_HPP_
