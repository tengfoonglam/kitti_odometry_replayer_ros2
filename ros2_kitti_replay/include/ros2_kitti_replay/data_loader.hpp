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
struct return_underlying_type_if_smart_pointer_else_itself
{
  using type = T;
};

template <class T>
struct return_underlying_type_if_smart_pointer_else_itself<std::shared_ptr<T>>
{
  using type = T;
};

template <class T>
struct return_underlying_type_if_smart_pointer_else_itself<std::weak_ptr<T>>
{
  using type = T;
};

template <class T>
struct return_underlying_type_if_smart_pointer_else_itself<std::unique_ptr<T>>
{
  using type = T;
};

template <class T>
struct is_weak_ptr : std::false_type
{
};

template <class T>
struct is_weak_ptr<std::weak_ptr<T>> : std::true_type
{
};

template <typename RT>
class DataLoader : public DataLoaderBase
{
  static_assert(!std::is_pointer_v<RT>, "A raw pointer return type is currently not supported.");
  static_assert(!is_weak_ptr<RT>::value, "A weak pointer return type is currently not supported.");

public:
  using DataType = typename return_underlying_type_if_smart_pointer_else_itself<RT>::type;
  using ReturnType = RT;
  using OptionalReturnType = std::optional<ReturnType>;

  static constexpr bool kReturnValueIsPtr = !std::is_same_v<DataType, ReturnType>;

  explicit DataLoader(const std::string & name) : DataLoaderBase(name) {}

  DataLoader(const std::string & name, rclcpp::Logger logger) : DataLoaderBase(name, logger) {}

  [[nodiscard]] OptionalReturnType get_data(const std::size_t idx)
  {
    if (!can_process_data(idx, __func__)) {
      return std::nullopt;
    }
    return get_data_internal(idx);
  }

protected:
  [[nodiscard]] virtual OptionalReturnType get_data_internal(const std::size_t idx) = 0;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__DATA_LOADER_HPP_
