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

/**
 * @brief Data loader for timestamps. Note that the passed in load_path is unused and the timestamps
 *        passed using the setup() method is already the data for this loader
 *
 */
class ClockDataLoader final : public DataLoader<rosgraph_msgs::msg::Clock>
{
public:
  /**
   * @brief Construct a new Clock Data Loader object
   *
   * @param name - Name of the data loader
   */
  explicit ClockDataLoader(const std::string & name);

  /**
   * @brief Construct a new Clock Data Loader object
   *
   * @param name - Name of the data loader
   * @param logger - Logger to use
   */
  ClockDataLoader(const std::string & name, rclcpp::Logger logger);

private:
  /**
   * @brief Setup implementation
   *
   * @param timestamps - Timestamps corresponding to each element
   * @param load_path - Path where the data is located
   * @return true - Data loader is ready
   * @return false - Otherwise
   */
  bool setup_internal(
    const Timestamps & timestamps, [[maybe_unused]] const std::filesystem::path & load_path) final;

  /**
   * @brief Prepare data implementation
   *
   * @param idx - Index of the data to be loaded
   * @return true - Successful
   * @return false - Otherwise
   */
  bool prepare_data_internal([[maybe_unused]] std::size_t idx) final;
  [[nodiscard]] OptionalReturnType get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__CLOCK_DATA_LOADER_HPP_
