#include "ros2_kitti_core/timer.hpp"

namespace r2k_core
{

Timer::Timer() : is_timing_(false), clock_(RCL_STEADY_TIME) {}

void Timer::start()
{
  start_time_ = clock_.now();
  is_timing_ = true;
}

rclcpp::Duration Timer::stop()
{
  if (!is_timing_) {
    return rclcpp::Duration{0, 0};
  }

  const auto end_time = clock_.now();
  const auto duration = end_time - start_time_;
  is_timing_ = false;

  return duration;
}

}  // namespace r2k_core
