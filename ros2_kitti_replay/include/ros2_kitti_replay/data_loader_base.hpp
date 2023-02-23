#ifndef ROS2_KITTI_REPLAY__DATA_LOADER_BASE_HPP_
#define ROS2_KITTI_REPLAY__DATA_LOADER_BASE_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_replay/timestamps.hpp"

namespace r2k_replay
{

class DataLoaderBase
{
public:
  explicit DataLoaderBase(const std::string & name);
  DataLoaderBase(const std::string & name, rclcpp::Logger logger);
  [[nodiscard]] constexpr bool ready() const noexcept { return ready_; };
  [[nodiscard]] const std::string & name() const noexcept { return name_; };
  [[nodiscard]] constexpr std::size_t number_packets() const noexcept
  {
    return ready() ? timestamps_.size() : std::size_t{0};
  };
  bool setup(const Timestamps & timestamps, const std::filesystem::path & load_path);
  bool prepare_data(const std::size_t idx);
  virtual ~DataLoaderBase();

protected:
  bool ready_{false};
  std::string name_;
  rclcpp::Logger logger_;
  Timestamps timestamps_;
  bool can_process_data(const std::size_t idx, const std::string & call_name);
  virtual bool setup_internal(
    const Timestamps & timestamps, const std::filesystem::path & load_path) = 0;
  virtual bool prepare_data_internal(const std::size_t idx) = 0;
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__DATA_LOADER_BASE_HPP_
