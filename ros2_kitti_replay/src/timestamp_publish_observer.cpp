#include "ros2_kitti_replay/timestamp_publish_observer.hpp"

namespace r2k_replay
{

bool TimestampPublishObserver::setup(
  TimestampPublishObserver::PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
  __attribute__((unused)) const std::filesystem::path & load_path)
{
  if (publisher_uq_ptr) {
    publisher_uq_ptr_ = std::move(publisher_uq_ptr);
    timestamps_ = timestamps;
    ready_ = true;
  }

  return false;
}

[[nodiscard]] bool TimestampPublishObserver::ready() { return ready_; }

bool TimestampPublishObserver::notify_send(const std::size_t idx)
{
  if (ready() && idx < number_readings()) {
    TimestampPublishObserver::Type clock_msg;
    clock_msg.set__clock(timestamps_.at(idx));
    publisher_uq_ptr_->publish(clock_msg);
  }

  return false;
}

bool TimestampPublishObserver::notify_prepare(const std::size_t idx)
{
  return (ready() && idx < number_readings());
}

[[nodiscard]] std::size_t TimestampPublishObserver::number_readings() const
{
  return timestamps_.size();
}

}  // namespace r2k_replay
