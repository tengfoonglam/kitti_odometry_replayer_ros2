#include "ros2_kitti_replay/point_cloud_data_loader.hpp"

#include <algorithm>

namespace r2k_replay
{

PointCloudDataLoader::PointCloudDataLoader(const std::string & name, const Header & header)
: DataLoader<PointCloudDataLoader::Type>(name), header_(header)
{
}

PointCloudDataLoader::PointCloudDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header)
: DataLoader<PointCloudDataLoader::Type>(name, logger), header_(header)
{
}

[[nodiscard]] bool PointCloudDataLoader::is_kitti_point_cloud_file(
  const std::filesystem::path & pc_path)
{
  const bool extension_match = pc_path.extension().string() == ".bin";
  const auto & stem = pc_path.stem().string();
  const bool number_char_match = stem.size() == 6;
  const bool stem_all_digits = std::all_of(stem.cbegin(), stem.cend(), ::isdigit);
  return extension_match && number_char_match && stem_all_digits;
}

bool PointCloudDataLoader::setup_internal(
  const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  ready_ = false;
  timestamps_ = timestamps;
  load_path_ = load_path;
  max_idx_ = 0;
  current_idx_opt_.reset();
  point_cloud_ptr_.reset();

  // Iterate within file and search for matching files, find max
  std::size_t temp_max_idx = 0;
  bool found_at_least_one_file = false;
  for (auto const & dir_entry : std::filesystem::directory_iterator{load_path}) {
    const auto current_path = dir_entry.path();
    if (is_kitti_point_cloud_file(current_path)) {
      found_at_least_one_file = true;
      const auto parsed_idx =
        static_cast<std::size_t>(std::atoll(current_path.stem().string().c_str()));
      temp_max_idx = std::max(temp_max_idx, parsed_idx);
    }
  }

  if (found_at_least_one_file) {
    max_idx_ = temp_max_idx;
    ready_ = true;
    RCLCPP_INFO(
      logger_, "%s point cloud data loader setup successfully. Largest point cloud index: %zu",
      name().c_str(), max_idx_);

  } else {
    RCLCPP_WARN(
      logger_, "%s point cloud data loader setup failed. Could not find point cloud files at %s",
      name().c_str(), load_path.string().c_str());
  }

  return ready();
}

[[nodiscard]] std::size_t PointCloudDataLoader::data_size() const
{
  return ready() ? std::min(timestamps_.size(), max_idx_ + 1) : std::size_t{0};
}

bool PointCloudDataLoader::prepare_data_internal(const std::size_t idx)
{
  if (idx >= data_size()) {
    return false;
  }

  const auto idx_unpadded = std::to_string(idx);
  auto idx_padded =
    std::string(
      kNumberDigitsFilename - std::min(kNumberDigitsFilename, idx_unpadded.length()), '0') +
    idx_unpadded;
  const auto file_to_load = load_path_ / (idx_padded + ".bin");
  point_cloud_ptr_ = load_point_cloud_from_file(file_to_load);

  if (point_cloud_ptr_) {
    point_cloud_ptr_->header = header_;
    point_cloud_ptr_->header.stamp = timestamps_.at(idx);
    current_idx_opt_.emplace(idx);
  }

  return static_cast<bool>(point_cloud_ptr_);
}

[[nodiscard]] PointCloudDataLoader::OptionalType PointCloudDataLoader::get_data_internal(
  const std::size_t idx)
{
  if (current_idx_opt_.has_value() && current_idx_opt_.value() == idx && point_cloud_ptr_) {
    return point_cloud_ptr_;
  }
  return std::nullopt;
}

}  // namespace r2k_replay
