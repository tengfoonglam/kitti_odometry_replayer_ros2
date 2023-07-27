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

class PoseDataLoader final : public DataLoader<TransformStamped>
{
public:
  using Header = std_msgs::msg::Header;

  PoseDataLoader(
    const std::string & name, const Header & header, const std::string & child_frame_id);
  PoseDataLoader(
    const std::string & name, rclcpp::Logger logger, const Header & header,
    const std::string & child_frame_id);

private:
  Header header_;
  std::string child_frame_id_;
  Transforms poses_;

  bool setup_internal(const Timestamps & timestamps, const std::filesystem::path & load_path) final;
  bool prepare_data_internal([[maybe_unused]] std::size_t idx) final;
  [[nodiscard]] OptionalReturnType get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__POSE_DATA_LOADER_HPP_
