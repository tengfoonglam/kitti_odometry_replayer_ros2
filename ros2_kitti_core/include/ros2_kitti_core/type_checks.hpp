#ifndef ROS2_KITTI_CORE__TYPE_CHECKS_HPP_
#define ROS2_KITTI_CORE__TYPE_CHECKS_HPP_

#include <memory>
#include <type_traits>

namespace r2k_core
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

template <class T>
struct is_shared_ptr : std::false_type
{
};

template <class T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
{
};
}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__TYPE_CHECKS_HPP_
