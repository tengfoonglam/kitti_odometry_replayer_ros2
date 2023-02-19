#include "ros2_kitti_replay/timestamp_publish_observer.hpp"

namespace r2k_replay
{

bool TimestampPublishObserver::start() {}

bool load(
  const std::filesystem::path & load_path,
  const TimestampPublishObserver::PublisherUqPtr publisher_uq_ptr)
{
  return false;
}

bool TimestampPublishObserver::stop() { return false; }

bool TimestampPublishObserver::notify_send(const std::size_t idx) { return false; }

bool TimestampPublishObserver::notify_prepare(const std::size_t idx) { return false; }

[[nodiscard]] std::size_t TimestampPublishObserver::number_readings() const
{
  return timestamps_.size();
}

}  // namespace r2k_replay
