#ifndef ROS2_KITTI_CORE__TIMER_HPP_
#define ROS2_KITTI_CORE__TIMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace r2k_core
{

class Timer
{
public:
  Timer();

  void start();

  rclcpp::Duration stop();

private:
  bool is_timing_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  rclcpp::Time start_time_;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__TIMER_HPP_
