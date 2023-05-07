#include "ros2_kitti_replay/point_cloud_data_loader.hpp"

#include <algorithm>

namespace r2k_replay
{

PointCloudDataLoader::PointCloudDataLoader(const std::string & name, const Header & header)
: DataLoader<PointCloudDataLoader::ReturnType>(name), header_(header)
{
}

PointCloudDataLoader::PointCloudDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header)
: DataLoader<PointCloudDataLoader::ReturnType>(name, logger), header_(header)
{
}

bool PointCloudDataLoader::setup_internal(
  const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  const auto pc_idx_opt = get_last_index_of_point_cloud_sequence(load_path);

  if (pc_idx_opt.has_value() && !timestamps.empty()) {
    timestamps_ = timestamps;
    load_path_ = load_path;
    max_idx_ = std::min(pc_idx_opt.value(), timestamps_.size() - 1);
    ready_ = true;
    RCLCPP_INFO(
      logger_, "%s point cloud data loader setup successfully. Number of point clouds: %zu",
      name().c_str(), max_idx_ + 1);
  } else {
    if (timestamps.empty()) {
      RCLCPP_WARN(
        logger_, "%s point cloud data loader setup failed. No timestamps provided", name().c_str());
    } else {
      RCLCPP_WARN(
        logger_, "%s point cloud data loader setup failed. Could not find point cloud files at %s",
        name().c_str(), load_path.string().c_str());
    }
  }

  return ready();
}

[[nodiscard]] std::size_t PointCloudDataLoader::data_size() const
{
  return ready() ? max_idx_ + 1 : std::size_t{0};
}

bool PointCloudDataLoader::prepare_data_internal(const std::size_t idx)
{
  point_cloud_ptr_ =
    load_point_cloud_from_file(from_index_to_point_cloud_file_path(idx, load_path_));

  if (point_cloud_ptr_) {
    point_cloud_ptr_->header = header_;
    point_cloud_ptr_->header.stamp = timestamps_.at(idx);
    current_idx_opt_ = idx;
  }

  return static_cast<bool>(point_cloud_ptr_);
}

[[nodiscard]] PointCloudDataLoader::OptionalReturnType PointCloudDataLoader::get_data_internal(
  const std::size_t idx)
{
  if (current_idx_opt_.has_value() && current_idx_opt_.value() == idx && point_cloud_ptr_) {
    return point_cloud_ptr_;
  }
  return std::nullopt;
}

}  // namespace r2k_replay
