#include "ros2_kitti_core/timer.hpp"

namespace r2k_core
{

Timer::Timer() : clock_(RCL_STEADY_TIME) {}

void Timer::start() { start_time_ = clock_.now(); }

rclcpp::Duration Timer::stop()
{
  const auto end_time = clock_.now();
  const auto duration = end_time - start_time_;
  return duration;
}

}  // namespace r2k_core
