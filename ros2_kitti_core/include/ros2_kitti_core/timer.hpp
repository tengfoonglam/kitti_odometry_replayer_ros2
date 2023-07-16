#ifndef ROS2_KITTI_CORE__TIMER_HPP_
#define ROS2_KITTI_CORE__TIMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace r2k_core
{

class Timer
{
public:
  Timer(
    const rclcpp::Logger & logger, const std::string & log_message, const std::size_t log_period_ms,
    const double time_scale_factor);

  void start();

  rclcpp::Duration stop_and_log();

private:
  rclcpp::Clock clock_;
  rclcpp::Logger logger_;
  std::string log_message_;
  std::size_t log_period_ms_;
  double display_time_scale_factor_;
  rclcpp::Time start_time_;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__TIMER_HPP_
