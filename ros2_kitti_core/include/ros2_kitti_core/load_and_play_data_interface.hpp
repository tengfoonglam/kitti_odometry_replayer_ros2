#ifndef ROS2_KITTI_CORE__LOAD_AND_PLAY_DATA_INTERFACE_HPP_
#define ROS2_KITTI_CORE__LOAD_AND_PLAY_DATA_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/play_data_interface_base.hpp"

namespace r2k_core
{

/**
 * @brief A PlayDataInterace where the DataLoader is the source of the data
 *
 * @tparam T - DataLoader
 * @tparam std::enable_if<std::is_base_of_v<DataLoader<typename T::ReturnType>, T>>::type
 */
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

  /**
   * @brief Construct a new Load And Play Data Interface object
   *
   * @param name - Name of interface
   * @param play_cb - Callback to trigger when play is called
   * @param data_loader_ptr - Pointer to the Data Loader that should be used
   */
  LoadAndPlayDataInterface(
    const std::string & name, const PlayCb & play_cb, DataLoaderUqPtr data_loader_ptr)
  : PlayDataInterfaceBase(name), play_cb_(play_cb), data_loader_ptr_(std::move(data_loader_ptr))
  {
  }

  /**
   * @brief Construct a new Load And Play Data Interface object, with universal reference for the
   * callback
   *
   * @param name - Name of interface
   * @param play_cb - Callback to trigger when play is called
   * @param data_loader_ptr - Pointer to the Data Loader that should be used
   */
  LoadAndPlayDataInterface(
    const std::string & name, PlayCb && play_cb, DataLoaderUqPtr data_loader_ptr)
  : PlayDataInterfaceBase(name), play_cb_(play_cb), data_loader_ptr_(std::move(data_loader_ptr))
  {
  }

  /**
   * @brief Interface is ready
   *
   * @return true - Ready
   * @return false - Otherwise
   */
  [[nodiscard]] virtual bool ready() const { return data_loader_ptr_->ready(); }

  /**
   * @brief Number of elements in the interface
   *
   * @return std::size_t - Number of elements
   */
  [[nodiscard]] virtual std::size_t data_size() const { return data_loader_ptr_->data_size(); }

  /**
   * @brief Prepare the data for a given index
   *
   * @param idx - Index of the data to prepare
   * @return true - Successful
   * @return false - Otherwise
   */
  bool virtual prepare(std::size_t idx) { return data_loader_ptr_->prepare_data(idx); }

  /**
   * @brief Play the data for a given index
   *
   * @param idx - Index of the data to play
   * @return true - Successful
   * @return false - Otherwise
   */
  bool play(std::size_t idx)
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

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__LOAD_AND_PLAY_DATA_INTERFACE_HPP_
