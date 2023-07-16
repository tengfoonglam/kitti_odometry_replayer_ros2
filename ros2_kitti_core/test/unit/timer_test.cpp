#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_core/timer.hpp>

TEST(TestTimer, NormalOperation)
{
  static constexpr std::size_t kSleepDurationNs = 1000000;
  static constexpr std::size_t kTimerToleranceNs = 200000;
  r2k_core::Timer timer;
  timer.start();
  std::this_thread::sleep_for(std::chrono::nanoseconds(kSleepDurationNs));
  const auto duration = timer.stop();
  const auto duration_ns = duration.nanoseconds();
  ASSERT_GE(duration_ns, static_cast<int64_t>(kSleepDurationNs));
  ASSERT_LT(duration_ns, static_cast<int64_t>(kSleepDurationNs + kTimerToleranceNs));
}
