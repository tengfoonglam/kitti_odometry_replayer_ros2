#ifndef ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_

#include <algorithm>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/point_cloud_utils.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"
#include "ros2_kitti_core/type_checks.hpp"

namespace r2k_core
{

template <typename RT>
class FolderDataLoader : public DataLoader<RT>
{
  static_assert(
    (is_shared_ptr<RT>::value || is_unique_ptr<RT>::value),
    "Currently FolderDataLoader only supports unique or shared pointer return types");

public:
  using Header = std_msgs::msg::Header;

  FolderDataLoader(const std::string & name, const Header & header)
  : DataLoader<RT>(name), header_(header)
  {
  }

  FolderDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header)
  : DataLoader<RT>(name, logger), header_(header)
  {
  }

  [[nodiscard]] std::size_t data_size() const override
  {
    return this->ready() ? max_idx_ + 1 : std::size_t{0};
  }

protected:
  Header header_;
  std::size_t max_idx_{};
  RT data_ptr_;
  std::optional<size_t> current_idx_opt_;
  std::filesystem::path load_path_;

  [[nodiscard]] virtual std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) = 0;

  [[nodiscard]] virtual RT load_data(
    const std::size_t idx, const std::filesystem::path & load_path) = 0;

  bool setup_internal(const Timestamps & timestamps, const std::filesystem::path & load_path) final
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

  bool prepare_data_internal(const std::size_t idx) final
  {
    data_ptr_ = this->load_data(idx, load_path_);

    if (data_ptr_) {
      data_ptr_->header = header_;
      data_ptr_->header.stamp = this->timestamps_.at(idx);
      current_idx_opt_ = idx;
    }

    return static_cast<bool>(data_ptr_);
  };

  [[nodiscard]] std::optional<RT> get_data_internal(const std::size_t idx) final
  {
    if (current_idx_opt_.has_value() && current_idx_opt_.value() == idx && data_ptr_) {
      return data_ptr_;
    }
    return std::nullopt;
  };
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
