#include "ros2_kitti_core/timer.hpp"

namespace r2k_core
{

Timer::Timer(
  rclcpp::Logger & logger, const std::string & log_message, const std::size_t period_ms,
  const double display_time_scale_factor)
: clock_(RCL_STEADY_TIME),
  logger_(logger),
  log_message_(log_message),
  period_ms_(period_ms),
  display_time_scale_factor_(display_time_scale_factor)
{
}

void Timer::start() { start_time_ = clock_.now(); }

rclcpp::Duration Timer::stop_and_log()
{
  const auto end_time = clock_.now();
  const auto duration = end_time - start_time_;
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, period_ms_, "%s: %f", log_message_.c_str(),
    duration.seconds() * display_time_scale_factor_);
  return duration;
}

}  // namespace r2k_core
