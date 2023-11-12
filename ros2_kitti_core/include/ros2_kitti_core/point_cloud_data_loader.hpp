#ifndef ROS2_KITTI_CORE__POINT_CLOUD_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__POINT_CLOUD_DATA_LOADER_HPP_

#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/folder_data_loader.hpp"
#include "ros2_kitti_core/point_cloud_utils.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

/**
 * @brief Data loaded that loads point clouds from a folder
 *
 */
class PointCloudDataLoader final : public FolderDataLoader<PointCloudMsg::SharedPtr>
{
public:
  using Header = std_msgs::msg::Header;

  /**
   * @brief Construct a new Point Cloud Data Loader object
   *
   * @param name - Name of the data loader
   * @param header - ROS Header msg to append to the data
   */
  PointCloudDataLoader(const std::string & name, const Header & header);

  /**
   * @brief Construct a new Point Cloud Data Loader object
   *
   * @param name - Name of the data loader
   * @param logger - Logger to use
   * @param header - ROS Header msg to append to the data
   */
  PointCloudDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);

private:
  /**
   * @brief In a folder, find the file with the largest index
   *
   * @param load_path - Folder to look for the file
   * @return std::optional<size_t> - Largest index, if available
   */
  [[nodiscard]] std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) final;

  /**
   * @brief Load the data corresponding to a particular index
   *
   * @param idx
   * @param load_path - Path to search for the data
   * @return PointCloudDataLoader::ReturnType- Loaded data. Empty pointer if load data was not
   * successful
   */
  [[nodiscard]] PointCloudDataLoader::ReturnType load_data(
    std::size_t idx, const std::filesystem::path & load_path) final;
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__POINT_CLOUD_DATA_LOADER_HPP_
