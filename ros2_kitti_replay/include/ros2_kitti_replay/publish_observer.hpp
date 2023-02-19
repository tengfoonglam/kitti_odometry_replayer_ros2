#ifndef ROS2_KITTI_REPLAY__REPLAY_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__REPLAY_OBSERVER_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace r2k_replay
{

template <typename T>
class PublishObserver
{
public:
  using Type = T;
  using PublisherUqPtr = std::unique_ptr<rclcpp::Publisher<T>>;

  virtual bool start() = 0;
  virtual bool load(
    const std::filesystem::path & load_path, const PublisherUqPtr publisher_uq_ptr) = 0;
  virtual bool stop() = 0;
  virtual bool notify_send(const std::size_t idx) = 0;
  virtual bool notify_prepare(const std::size_t idx) = 0;
  [[nodiscard]] virtual std::size_t number_readings() const = 0;
  virtual ~PublishObserver() {}

protected:
  PublisherUqPtr publisher_uq_ptr_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__REPLAY_OBSERVER_HPP_
