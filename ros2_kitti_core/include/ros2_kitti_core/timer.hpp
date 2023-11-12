#ifndef ROS2_KITTI_CORE__TIMER_HPP_
#define ROS2_KITTI_CORE__TIMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace r2k_core
{

class Timer
{
public:
  /**
   * @brief Construct a new Timer object
   *
   */
  Timer();

  /**
   * @brief Start the timer
   *
   */
  void start();

  /**
   * @brief Stop the timer, return the time elapsed
   *
   * @return rclcpp::Duration - Elapsted time
   */
  [[nodiscard]] rclcpp::Duration stop();

private:
  bool is_timing_;
  rclcpp::Clock clock_;
  rclcpp::Time start_time_;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__TIMER_HPP_
