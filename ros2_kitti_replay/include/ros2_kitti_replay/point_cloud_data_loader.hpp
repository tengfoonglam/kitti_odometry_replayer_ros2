#ifndef ROS2_KITTI_REPLAY__POINT_CLOUD_DATA_LOADER_HPP_
#define ROS2_KITTI_REPLAY__POINT_CLOUD_DATA_LOADER_HPP_

#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_replay/data_loader.hpp"
#include "ros2_kitti_replay/point_cloud_utils.hpp"
#include "ros2_kitti_replay/timestamp_utils.hpp"

namespace r2k_replay
{

class PointCloudDataLoader final : public DataLoader<r2k_replay::PointCloudMsg::SharedPtr>
{
public:
  using Header = std_msgs::msg::Header;

  explicit PointCloudDataLoader(const std::string & name, const Header & header);
  PointCloudDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);
  [[nodiscard]] std::size_t data_size() const final;

private:
  Header header_;
  std::size_t max_idx_{};
  PointCloudDataLoader::Type point_cloud_ptr_;
  std::optional<size_t> current_idx_opt_;
  std::filesystem::path load_path_;

  bool setup_internal(const Timestamps & timestamps, const std::filesystem::path & load_path) final;
  bool prepare_data_internal(const std::size_t idx) final;
  [[nodiscard]] OptionalType get_data_internal(const std::size_t idx) final;
};

}  // namespace r2k_replay
#endif  // ROS2_KITTI_REPLAY__POINT_CLOUD_DATA_LOADER_HPP_
