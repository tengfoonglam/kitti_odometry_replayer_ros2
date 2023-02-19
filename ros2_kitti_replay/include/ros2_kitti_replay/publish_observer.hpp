#ifndef ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

template <typename T>
class PublishObserver
{
public:
  using Type = T;
  using PublisherUqPtr = std::unique_ptr<rclcpp::Publisher<T>>;

  explicit PublishObserver(const std::string & name)
  : name_(name), logger_(rclcpp::get_logger(name))
  {
  }

  PublishObserver(const std::string & name, rclcpp::Logger logger) : name_(name), logger_(logger) {}

  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] const std::string & name() const { return name_; }
  [[nodiscard]] std::size_t number_readings() const { return timestamps_.size(); }

  bool setup(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    const std::filesystem::path & load_path)
  {
    if (!ready()) {
      RCLCPP_WARN(
        logger_, "{} Publish Observer already ready and setup, ignoring setup() call", name());
      return false;
    }
    return setup_internal(std::move(publisher_uq_ptr), timestamps, load_path);
  }

  bool notify_prepare(const std::size_t idx)
  {
    if (!can_process_notify_request(idx, __func__)) {
      return false;
    }
    return notify_prepare_internal(idx);
  }

  bool notify_send(const std::size_t idx)
  {
    if (!can_process_notify_request(idx, __func__)) {
      return false;
    }
    return notify_send_internal(idx);
  }

  virtual ~PublishObserver() {}

protected:
  bool ready_{false};
  std::string name_;
  rclcpp::Logger logger_;
  PublisherUqPtr publisher_uq_ptr_;
  Timestamps timestamps_;

  bool can_process_notify_request(const std::size_t idx, const std::string & call_name)
  {
    if (ready() && idx < timestamps_.size()) {
      return true;
    } else {
      RCLCPP_WARN(
        logger_,
        "{} Publish Observer could not process {} with the requested index {}. Debug info - ready: "
        "{}, number readings: {}",
        name(), call_name, idx, ready(), number_readings());
      return false;
    }
  }

  virtual bool setup_internal(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    const std::filesystem::path & load_path) = 0;

  virtual bool notify_send_internal(const std::size_t idx) = 0;

  virtual bool notify_prepare_internal(const std::size_t idx) = 0;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
