#ifndef ROS2_KITTI_REPLAY__OBSERVER_BASE_HPP_
#define ROS2_KITTI_REPLAY__OBSERVER_BASE_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

class ObserverBase
{
public:
  explicit ObserverBase(const std::string & name);
  ObserverBase(const std::string & name, rclcpp::Logger logger);
  [[nodiscard]] bool ready() const;
  [[nodiscard]] const std::string & name() const;
  [[nodiscard]] std::size_t number_readings() const;
  bool setup(const Timestamps & timestamps, const std::filesystem::path & load_path);
  bool notify_prepare(const std::size_t idx);
  bool notify_send(const std::size_t idx);
  virtual ~ObserverBase();

protected:
  bool ready_{false};
  std::string name_;
  rclcpp::Logger logger_;
  Timestamps timestamps_;
  bool can_process_notify_request(const std::size_t idx, const std::string & call_name);
  virtual bool setup_internal(
    const Timestamps & timestamps, const std::filesystem::path & load_path) = 0;
  virtual bool notify_send_internal(const std::size_t idx) = 0;
  virtual bool notify_prepare_internal(const std::size_t idx) = 0;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__OBSERVER_BASE_HPP_
