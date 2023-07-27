#include "ros2_kitti_core/folder_data_loader.hpp"

#include "ros2_kitti_core/image_utils.hpp"
#include "ros2_kitti_core/point_cloud_utils.hpp"

namespace r2k_core
{

template <typename RT>
FolderDataLoader<RT>::FolderDataLoader(const std::string & name, const Header & header)
: DataLoader<RT>(name), header_(header)
{
}

template <typename RT>
FolderDataLoader<RT>::FolderDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header)
: DataLoader<RT>(name, logger), header_(header)
{
}

template <typename RT>
std::size_t FolderDataLoader<RT>::data_size() const
{
  return this->ready() ? max_idx_ + 1 : std::size_t{0};
}

template <typename RT>
bool FolderDataLoader<RT>::setup_internal(
  const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  const auto data_idx_opt = get_last_index_of_sequence(load_path);

  if (data_idx_opt.has_value() && !timestamps.empty()) {
    this->timestamps_ = timestamps;
    load_path_ = load_path;
    max_idx_ = std::min(data_idx_opt.value(), this->timestamps_.size() - 1);
    this->ready_ = true;
    RCLCPP_INFO(
      this->logger_, "%s point cloud data loader setup successful. Number of point clouds: %zu",
      this->name().c_str(), max_idx_ + 1);
  } else {
    if (timestamps.empty()) {
      RCLCPP_WARN(
        this->logger_, "%s point cloud data loader setup failed. No timestamps provided",
        this->name().c_str());
    } else {
      RCLCPP_WARN(
        this->logger_,
        "%s point cloud data loader setup failed. Could not find point cloud files at %s",
        this->name().c_str(), load_path.string().c_str());
    }
  }

  return this->ready();
};

template <typename RT>
bool FolderDataLoader<RT>::prepare_data_internal(std::size_t idx)
{
  data_ptr_ = this->load_data(idx, load_path_);

  if (data_ptr_) {
    data_ptr_->header = header_;
    data_ptr_->header.stamp = this->timestamps_.at(idx);
    current_idx_opt_ = idx;
  }

  return static_cast<bool>(data_ptr_);
};

template <typename RT>
std::optional<RT> FolderDataLoader<RT>::get_data_internal(std::size_t idx)
{
  if (current_idx_opt_.has_value() && current_idx_opt_.value() == idx && data_ptr_) {
    return data_ptr_;
  }
  return std::nullopt;
};

// Explicit template instantiation so we can keep methods in source file
template class FolderDataLoader<PointCloudMsg::SharedPtr>;
template class FolderDataLoader<ImageMsg::SharedPtr>;

}  // namespace r2k_core
