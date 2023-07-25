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

class PointCloudDataLoader final : public FolderDataLoader<PointCloudMsg::SharedPtr>
{
public:
  using Header = std_msgs::msg::Header;

  PointCloudDataLoader(const std::string & name, const Header & header);
  PointCloudDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);

private:
  [[nodiscard]] std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) final;

  [[nodiscard]] PointCloudDataLoader::ReturnType load_data(
    const std::size_t idx, const std::filesystem::path & load_path) final;
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__POINT_CLOUD_DATA_LOADER_HPP_
