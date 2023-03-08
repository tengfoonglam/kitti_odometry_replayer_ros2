#ifndef ROS2_KITTI_REPLAY__PLAY_DATA_CALLBACK_BASE_HPP_
#define ROS2_KITTI_REPLAY__PLAY_DATA_CALLBACK_BASE_HPP_

#include <cstdint>
#include <string>

class PlayDataCallbackBase
{
public:
  explicit PlayDataCallbackBase(const std::string & name) : name_(name) {}
  [[nodiscard]] const std::string & name() const noexcept { return name_; }
  [[nodiscard]] virtual bool ready() const = 0;
  [[nodiscard]] virtual std::size_t data_size() const = 0;
  virtual bool prepare(const std::size_t idx) const = 0;
  virtual bool play(const std::size_t idx) const = 0;
  virtual ~PlayDataCallbackBase() {}

protected:
  std::string name_;
};

#endif  // ROS2_KITTI_REPLAY__PLAY_DATA_CALLBACK_BASE_HPP_
