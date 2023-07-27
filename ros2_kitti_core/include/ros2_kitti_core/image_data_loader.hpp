#ifndef ROS2_KITTI_CORE__IMAGE_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__IMAGE_DATA_LOADER_HPP_

#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/folder_data_loader.hpp"
#include "ros2_kitti_core/image_utils.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"

namespace r2k_core
{

class ImageDataLoader final : public FolderDataLoader<ImageMsg::SharedPtr>
{
public:
  using Header = std_msgs::msg::Header;

  ImageDataLoader(const std::string & name, const Header & header);
  ImageDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);

private:
  [[nodiscard]] std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) final;

  [[nodiscard]] ImageDataLoader::ReturnType load_data(
    std::size_t idx, const std::filesystem::path & load_path) final;
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__IMAGE_DATA_LOADER_HPP_
