#ifndef ROS2_KITTI_CORE__DATA_LOADER_BASE_HPP_
#define ROS2_KITTI_CORE__DATA_LOADER_BASE_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

/**
 * @brief Base class for Data Loader
 *
 */
class DataLoaderBase
{
public:
  /**
   * @brief Construct a new Data Loader Base object. A data logger with the provided name will be
   *        created
   *
   * @param name - Name of the data loader
   */
  explicit DataLoaderBase(const std::string & name);

  /**
   * @brief Construct a new Data Loader Base object
   *
   * @param name - Name of the data loader
   * @param logger - Logger to use
   */
  DataLoaderBase(const std::string & name, rclcpp::Logger logger);

  /**
   * @brief Whether or not the data loader is ready to be used
   *
   * @return true - Ready
   * @return false - Otherwise
   */
  [[nodiscard]] constexpr bool ready() const noexcept { return ready_; };

  /**
   * @brief Name of the logger
   *
   * @return const std::string& - name
   */
  [[nodiscard]] const std::string & name() const noexcept { return name_; };

  /**
   * @brief Number of elements in the data loader
   *
   * @return std::size_t - Number of elements
   */
  [[nodiscard]] virtual std::size_t data_size() const;

  /**
   * @brief Setup the data loader
   *
   * @param timestamps - Timestamps corresponding to each element
   * @param load_path - Path where the data is located
   * @return true - Data loader is ready
   * @return false - Otherwise
   */
  bool setup(const Timestamps & timestamps, const std::filesystem::path & load_path);

  /**
   * @brief Prepare the data for a given index
   *
   * @param idx - Index of the data to prepare
   * @return true - Successful
   * @return false - Otherwise
   */
  bool prepare_data(std::size_t idx);

  /**
   * @brief Destroy the Data Loader Base object
   *
   */
  virtual ~DataLoaderBase() = default;

protected:
  bool ready_{false};
  std::string name_;
  rclcpp::Logger logger_;
  Timestamps timestamps_;

  /**
   * @brief Check if the data can be processed. Index must be valid and data loader must be ready
   *
   * @param idx - Index of the data to be loaded
   * @param call_name - Name of the function to be shown on the log message
   * @return true - Data can be processed
   * @return false - Otherwise
   */
  bool can_process_data(std::size_t idx, const std::string & call_name);

  /**
   * @brief Setup implementation
   *
   * @param timestamps - Timestamps corresponding to each element
   * @param load_path - Path where the data is located
   * @return true - Data loader is ready
   * @return false - Otherwise
   */
  virtual bool setup_internal(
    const Timestamps & timestamps, const std::filesystem::path & load_path) = 0;

  /**
   * @brief Prepare data implementation
   *
   * @param idx - Index of the data to be loaded
   * @return true - Successful
   * @return false - Otherwise
   */
  virtual bool prepare_data_internal(std::size_t idx) = 0;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__DATA_LOADER_BASE_HPP_
