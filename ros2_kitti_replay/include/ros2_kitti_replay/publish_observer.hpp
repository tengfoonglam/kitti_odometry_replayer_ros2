#ifndef ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
#define ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_replay/observer_base.hpp"
#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

template <typename T>
class PublishObserver : public ObserverBase
{
public:
  using Type = T;
  using PublisherUqPtr = std::unique_ptr<rclcpp::Publisher<T>>;

  explicit PublishObserver(const std::string & name) : ObserverBase(name) {}

  PublishObserver(const std::string & name, rclcpp::Logger logger) : ObserverBase(name, logger) {}

  PublishObserver(const PublishObserver & other) = delete;
  PublishObserver & operator=(PublishObserver other) = delete;

  bool setup(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    const std::filesystem::path & load_path)
  {
    if (!ObserverBase::setup(timestamps, load_path)) {
      return false;
    }
    return setup_internal(std::move(publisher_uq_ptr), timestamps, load_path);
  }

protected:
  PublisherUqPtr publisher_uq_ptr_;

  virtual bool setup_internal(
    PublisherUqPtr publisher_uq_ptr, const Timestamps & timestamps,
    const std::filesystem::path & load_path) = 0;

private:
  using ObserverBase::setup;

  bool setup_internal(
    [[maybe_unused]] const Timestamps & timestamps,
    [[maybe_unused]] const std::filesystem::path & load_path) final
  {
    return true;
  }
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__PUBLISH_OBSERVER_HPP_
