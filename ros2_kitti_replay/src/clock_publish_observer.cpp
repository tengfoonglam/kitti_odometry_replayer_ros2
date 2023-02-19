#include "ros2_kitti_replay/clock_publish_observer.hpp"

namespace r2k_replay
{

ClockPublishObserver::ClockPublishObserver(const std::string & name)
: PublishObserver<rosgraph_msgs::msg::Clock>(name)
{
}

ClockPublishObserver::ClockPublishObserver(const std::string & name, rclcpp::Logger logger)
: PublishObserver<rosgraph_msgs::msg::Clock>(name, logger)
{
}

bool ClockPublishObserver::setup_internal(
  ClockPublishObserver::PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
  __attribute__((unused)) const std::filesystem::path & load_path)
{
  if (publisher_uq_ptr) {
    publisher_uq_ptr_ = std::move(publisher_uq_ptr);
    timestamps_ = timestamps;
    ready_ = true;
  }

  return ready();
}

bool ClockPublishObserver::notify_send_internal(const std::size_t idx)
{
  ClockPublishObserver::Type clock_msg;
  clock_msg.set__clock(timestamps_.at(idx));
  publisher_uq_ptr_->publish(clock_msg);
  return true;
}

bool ClockPublishObserver::notify_prepare_internal([[maybe_unused]] const std::size_t idx)
{
  return true;
}

}  // namespace r2k_replay
