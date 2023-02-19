#ifndef ROS2_KITTI_REPLAY__TIMESTAMP_PUBLISH_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__TIMESTAMP_PUBLISH_OBSERVER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

#include "ros2_kitti_replay/publish_observer.hpp"
#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

class TimestampPublishObserver : public PublishObserver<rosgraph_msgs::msg::Clock>
{
public:
  explicit TimestampPublishObserver(const std::string & name);
  TimestampPublishObserver(const std::string & name, rclcpp::Logger logger);

private:
  bool setup_internal(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    [[maybe_unused]] const std::filesystem::path & load_path) final;
  bool notify_prepare_internal([[maybe_unused]] const std::size_t idx) final;
  bool notify_send_internal(const std::size_t idx) final;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__TIMESTAMP_PUBLISH_OBSERVER_HPP_
