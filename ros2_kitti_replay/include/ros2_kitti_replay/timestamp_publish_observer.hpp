#ifndef ROS2_KITTI_REPLAY__TIMESTAMP_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__TIMESTAMP_OBSERVER_HPP_

#include <rosgraph_msgs/msg/clock.hpp>

#include "ros2_kitti_replay/publish_observer.hpp"
#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

class TimestampPublishObserver : public PublishObserver<rosgraph_msgs::msg::Clock>
{
public:
  bool start() final;
  bool load(const std::filesystem::path & load_path, const PublisherUqPtr publisher_uq_ptr) final;
  bool stop() final;
  bool notify_send(const std::size_t idx) final;
  bool notify_prepare(const std::size_t idx) final;
  [[nodiscard]] std::size_t number_readings() const final;

private:
  Timestamps timestamps_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__TIMESTAMP_OBSERVER_HPP_
