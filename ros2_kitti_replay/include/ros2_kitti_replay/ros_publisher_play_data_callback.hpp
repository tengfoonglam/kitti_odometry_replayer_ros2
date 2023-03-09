#ifndef ROS2_KITTI_REPLAY__ROS_PUBLISHER_PLAY_DATA_CALLBACK_HPP_
#define ROS2_KITTI_REPLAY__ROS_PUBLISHER_PLAY_DATA_CALLBACK_HPP_

#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_replay/data_loader.hpp"
#include "ros2_kitti_replay/play_data_callback_base.hpp"

namespace r2k_replay
{
template <typename T>
class ROSPublisherPlayDataCallback final : public PlayDataCallbackBase
{
public:
  ROSPublisherPlayDataCallback(
    const std::string & name, std::shared_ptr<rclcpp::Publisher<T>> publisher_ptr,
    std::unique_ptr<DataLoader<T>> data_loader_ptr_)
  : PlayDataCallbackBase(name),
    publisher_ptr_(publisher_ptr),
    data_loader_ptr_(std::move(data_loader_ptr_))
  {
  }

  [[nodiscard]] bool ready() const final { return data_loader_ptr_->ready(); }

  [[nodiscard]] std::size_t data_size() const final { return data_loader_ptr_->data_size(); }

  bool prepare(const std::size_t idx) final { return data_loader_ptr_->prepare_data(idx); }

  bool play(const std::size_t idx)
  {
    const auto data_opt = data_loader_ptr_->get_data(idx);
    if (data_opt.has_value()) {
      publisher_ptr_->publish(data_opt.value());
      return true;
    }
    return false;
  }

private:
  std::shared_ptr<rclcpp::Publisher<T>> publisher_ptr_;
  std::unique_ptr<DataLoader<T>> data_loader_ptr_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__PLAY_DATA_CALLBACK_BASE_HPP_
