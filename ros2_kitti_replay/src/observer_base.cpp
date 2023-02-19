#include "ros2_kitti_replay/observer_base.hpp"

namespace r2k_replay
{

ObserverBase::ObserverBase(const std::string & name)
: name_(name), logger_(rclcpp::get_logger(name))
{
}

ObserverBase::ObserverBase(const std::string & name, rclcpp::Logger logger)
: name_(name), logger_(logger)
{
}

[[nodiscard]] bool ObserverBase::ready() const { return ready_; }
[[nodiscard]] const std::string & ObserverBase::name() const { return name_; }
[[nodiscard]] std::size_t ObserverBase::number_readings() const { return timestamps_.size(); }

bool ObserverBase::setup(const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  if (!ready()) {
    RCLCPP_WARN(
      logger_, "%s Observer already ready and setup, ignoring setup() call", name().c_str());
    return false;
  }
  return setup_internal(timestamps, load_path);
}

bool ObserverBase::notify_prepare(const std::size_t idx)
{
  if (!can_process_notify_request(idx, __func__)) {
    return false;
  }
  return notify_prepare_internal(idx);
}

bool ObserverBase::notify_send(const std::size_t idx)
{
  if (!can_process_notify_request(idx, __func__)) {
    return false;
  }
  return notify_send_internal(idx);
}

ObserverBase::~ObserverBase() {}

bool ObserverBase::can_process_notify_request(const std::size_t idx, const std::string & call_name)
{
  if (ready() && idx < timestamps_.size()) {
    return true;
  } else {
    RCLCPP_WARN(
      logger_,
      "%s Observer could not process %s with the requested index %lu. Debug info - ready: %s"
      ", number readings: %lu",
      name().c_str(), call_name.c_str(), idx, ready() ? "true" : "false", number_readings());
    return false;
  }
}

}  // namespace r2k_replay
