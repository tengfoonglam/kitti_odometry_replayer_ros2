#ifndef ROS2_KITTI_CORE__CLOCK_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__CLOCK_DATA_LOADER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <filesystem>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

class ClockDataLoader final : public DataLoader<rosgraph_msgs::msg::Clock>
{
public:
  explicit ClockDataLoader(const std::string & name);
  ClockDataLoader(const std::string & name, rclcpp::Logger logger);

private:
  bool setup_internal(
    const Timestamps & timestamps, [[maybe_unused]] const std::filesystem::path & load_path) final;
  bool prepare_data_internal([[maybe_unused]] std::size_t idx) final;
  [[nodiscard]] OptionalReturnType get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__CLOCK_DATA_LOADER_HPP_
