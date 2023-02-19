#ifndef ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

template <typename T>
class PublishObserver
{
public:
  using Type = T;
  using PublisherUqPtr = std::unique_ptr<rclcpp::Publisher<T>>;

  virtual bool setup(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    const std::filesystem::path & load_path) = 0;
  [[nodiscard]] virtual bool ready() = 0;
  virtual bool notify_send(const std::size_t idx) = 0;
  virtual bool notify_prepare(const std::size_t idx) = 0;
  [[nodiscard]] virtual std::size_t number_readings() const = 0;
  virtual ~PublishObserver() {}

protected:
  PublisherUqPtr publisher_uq_ptr_;
  Timestamps timestamps_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
