#ifndef ROS2_KITTI_REPLAY__DATA_LOADER_HPP_
#define ROS2_KITTI_REPLAY__DATA_LOADER_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <utility>

#include "ros2_kitti_replay/data_loader_base.hpp"
#include "ros2_kitti_replay/timestamp_utils.hpp"

namespace r2k_replay
{

template <class T>
struct return_underlying_type_if_smart_pointer
{
  using type = T;
};

template <class T>
struct return_underlying_type_if_smart_pointer<std::shared_ptr<T>>
{
  using type = T;
};

template <class T>
struct return_underlying_type_if_smart_pointer<std::unique_ptr<T>>
{
  using type = T;
};

template <typename T>
class DataLoader : public DataLoaderBase
{
public:
  using DataType = typename return_underlying_type_if_smart_pointer<T>::type;
  using Type = T;
  using OptionalType = std::optional<Type>;

  static constexpr bool kReturnValueIsPtr = !std::is_same_v<DataType, Type>;

  explicit DataLoader(const std::string & name) : DataLoaderBase(name) {}

  DataLoader(const std::string & name, rclcpp::Logger logger) : DataLoaderBase(name, logger) {}

  [[nodiscard]] OptionalType get_data(const std::size_t idx)
  {
    if (!can_process_data(idx, __func__)) {
      return std::nullopt;
    }
    return get_data_internal(idx);
  }

protected:
  [[nodiscard]] virtual OptionalType get_data_internal(const std::size_t idx) = 0;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__DATA_LOADER_HPP_
