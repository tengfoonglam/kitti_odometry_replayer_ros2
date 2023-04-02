#include <ros2_kitti_replay/timestamps.hpp>

namespace r2k_replay_test
{
[[nodiscard]] inline auto generate_test_timestamps(const std::size_t start, const std::size_t end)
{
  r2k_replay::Timestamps output;
  for (std::size_t i = start; i <= end; i++) {
    output.emplace_back(i, 0);
  }
  return output;
}
}  // namespace r2k_replay_test
