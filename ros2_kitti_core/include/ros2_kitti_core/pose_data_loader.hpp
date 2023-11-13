#ifndef ROS2_KITTI_CORE__POSE_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__POSE_DATA_LOADER_HPP_

#include <cstdint>
#include <filesystem>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/pose_utils.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

/**
 * @brief Data loader that loads poses written into a single file
 *
 */
class PoseDataLoader final : public DataLoader<TransformStamped>
{
public:
  using Header = std_msgs::msg::Header;

  /**
   * @brief Construct a new Pose Data Loader object
   *
   * @param name - Name of the data loader
   * @param header - ROS Header msg to append to the data
   * @param child_frame_id - Child frame id of the poses
   */
  PoseDataLoader(
    const std::string & name, const Header & header, const std::string & child_frame_id);

  /**
   * @brief Construct a new Pose Data Loader object
   *
   * @param name - Name of the data loader
   * @param logger - Logger to use
   * @param header - ROS Header msg to append to the data
   * @param child_frame_id - Child frame id of the poses
   */
  PoseDataLoader(
    const std::string & name, rclcpp::Logger logger, const Header & header,
    const std::string & child_frame_id);

private:
  Header header_;
  std::string child_frame_id_;
  Transforms poses_;

  /**
   * @brief Setup implementation
   *
   * @param timestamps - Timestamps corresponding to each element
   * @param load_path - Path where pose data .txt file is located
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
  bool prepare_data_internal([[maybe_unused]] std::size_t idx) final;

  /**
   * @brief Get the data given an index
   *
   * @param idx - Index of the data to
   * @return OptionalReturnType - Data, std::nullopt if failed
   */
  [[nodiscard]] OptionalReturnType get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POSE_DATA_LOADER_HPP_
