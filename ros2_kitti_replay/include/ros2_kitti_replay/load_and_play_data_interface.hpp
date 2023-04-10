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
template <
  typename T,
  typename =
    typename std::enable_if<std::is_base_of_v<DataLoader<typename T::ReturnType>, T>>::type>
class LoadAndPlayDataInterface : public PlayDataInterfaceBase
{
public:
  using DataLoaderT = T;
  using DataLoaderUqPtr = std::unique_ptr<DataLoaderT>;
  using PlayCb = std::function<bool(const typename DataLoaderT::DataType &)>;

  LoadAndPlayDataInterface(
    const std::string & name, const PlayCb & play_cb, DataLoaderUqPtr data_loader_ptr)
  : PlayDataInterfaceBase(name), play_cb_(play_cb), data_loader_ptr_(std::move(data_loader_ptr))
  {
  }

  LoadAndPlayDataInterface(
    const std::string & name, PlayCb && play_cb, DataLoaderUqPtr data_loader_ptr)
  : PlayDataInterfaceBase(name), play_cb_(play_cb), data_loader_ptr_(std::move(data_loader_ptr))
  {
  }

  [[nodiscard]] virtual bool ready() const { return data_loader_ptr_->ready(); }

  [[nodiscard]] virtual std::size_t data_size() const { return data_loader_ptr_->data_size(); }

  bool virtual prepare(const std::size_t idx) { return data_loader_ptr_->prepare_data(idx); }

  bool play(const std::size_t idx)
  {
    const auto data_opt = data_loader_ptr_->get_data(idx);
    if (!data_opt.has_value()) {
      return false;
    }

    if constexpr (DataLoaderT::kReturnValueIsPtr) {
      return play_cb_(*(data_opt.value()));
    } else {
      return play_cb_(data_opt.value());
    }
  }

private:
  PlayCb play_cb_;
  DataLoaderUqPtr data_loader_ptr_;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__LOAD_AND_PLAY_DATA_INTERFACE_HPP_
