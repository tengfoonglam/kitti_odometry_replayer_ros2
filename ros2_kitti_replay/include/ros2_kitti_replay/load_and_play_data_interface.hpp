#ifndef ROS2_KITTI_REPLAY__LOAD_AND_PLAY_DATA_INTERFACE_HPP_
#define ROS2_KITTI_REPLAY__LOAD_AND_PLAY_DATA_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ros2_kitti_replay/data_loader.hpp"
#include "ros2_kitti_replay/play_data_interface_base.hpp"

namespace r2k_replay
{
template <typename T>
class LoadAndPlayDataInterface : public PlayDataInterfaceBase
{
public:
  using PlayCb = std::function<bool(const T &)>;

  LoadAndPlayDataInterface(
    const std::string & name, const PlayCb & play_cb,
    std::unique_ptr<DataLoader<T>> data_loader_ptr)
  : PlayDataInterfaceBase(name), play_cb_(play_cb), data_loader_ptr_(std::move(data_loader_ptr))
  {
  }

  [[nodiscard]] virtual bool ready() const { return data_loader_ptr_->ready() && play_cb_; }

  [[nodiscard]] virtual std::size_t data_size() const { return data_loader_ptr_->data_size(); }

  bool virtual prepare(const std::size_t idx) { return data_loader_ptr_->prepare_data(idx); }

  bool play(const std::size_t idx)
  {
    const auto data_opt = data_loader_ptr_->get_data(idx);
    if (data_opt.has_value() && play_cb_) {
      return play_cb_(data_opt.value());
    }
    return false;
  }

private:
  PlayCb play_cb_;
  std::unique_ptr<DataLoader<T>> data_loader_ptr_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__LOAD_AND_PLAY_DATA_INTERFACE_HPP_
