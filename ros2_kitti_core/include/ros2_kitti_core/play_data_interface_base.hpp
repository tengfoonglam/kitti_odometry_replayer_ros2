#ifndef ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_
#define ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_

#include <cstdint>
#include <string>

namespace r2k_core
{

class PlayDataInterfaceBase
{
public:
  explicit PlayDataInterfaceBase(const std::string & name) : name_(name) {}
  [[nodiscard]] const std::string & name() const noexcept { return name_; }
  [[nodiscard]] virtual bool ready() const = 0;
  [[nodiscard]] virtual std::size_t data_size() const = 0;
  virtual bool prepare(const std::size_t idx) = 0;
  virtual bool play(const std::size_t idx) = 0;
  virtual ~PlayDataInterfaceBase() = default;

protected:
  std::string name_;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__PLAY_DATA_INTERFACE_BASE_HPP_
