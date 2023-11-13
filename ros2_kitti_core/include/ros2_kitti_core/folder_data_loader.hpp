#ifndef ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_

#include <algorithm>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"
#include "ros2_kitti_core/type_checks.hpp"

namespace r2k_core
{

/**
 * @brief Data loader that loads data where there are multiple files in a folder
 *
 * @tparam RT - Return type of the loader. Must be either a shared or unique pointer
 */
template <typename RT>
class FolderDataLoader : public DataLoader<RT>
{
  static_assert(
    (is_shared_ptr<RT>::value || is_unique_ptr<RT>::value),
    "Currently FolderDataLoader only supports unique or shared pointer return types");

public:
  using Header = std_msgs::msg::Header;

  /**
   * @brief Construct a new Folder Data Loader object
   *
   * @param name - Name of the data loader
   * @param header - ROS Header msg to append to the data
   */
  FolderDataLoader(const std::string & name, const Header & header);

  /**
   * @brief Construct a new Folder Data Loader object
   *
   * @param name - Name of the data loader
   * @param logger - Logger to use
   * @param header - ROS Header msg to append to the data
   */
  FolderDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);

  /**
   * @brief Total number of data elements available
   *
   * @return std::size_t
   */
  [[nodiscard]] std::size_t data_size() const final;

protected:
  Header header_;
  std::size_t max_idx_{0};
  RT data_ptr_;
  std::optional<size_t> current_idx_opt_;
  std::filesystem::path load_path_;

  /**
   * @brief In a folder, find the file with the largest index
   *
   * @param load_path - Folder to look for the file
   * @return std::optional<size_t> - Largest index, if available
   */
  [[nodiscard]] virtual std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) = 0;

  /**
   * @brief Load the data corresponding to a particular index
   *
   * @param idx
   * @param load_path - Path to search for the data
   * @return RT - Loaded data. Empty pointer if load data was not successful
   */
  [[nodiscard]] virtual RT load_data(std::size_t idx, const std::filesystem::path & load_path) = 0;

  /**
   * @brief Setup implementation
   *
   * @param timestamps - Timestamps corresponding to each element
   * @param load_path - Path where the data is located
   * @return true - Data loader is ready
   * @return false - Otherwise
   */
  bool setup_internal(const Timestamps & timestamps, const std::filesystem::path & load_path) final;

  /**
   * @brief Prepare data implementation
   *
   * @param idx - Index of the data to be loaded
   * @return true - Successful
   * @return false - Otherwise
   */
  bool prepare_data_internal(std::size_t idx) final;

  /**
   * @brief Get the data given an index
   *
   * @param idx - Index of the data to
   * @return std::optional<RT> - Data, std::nullopt if failed
   */
  std::optional<RT> get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
