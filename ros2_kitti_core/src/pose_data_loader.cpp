#include "ros2_kitti_core/pose_data_loader.hpp"

#include <algorithm>
namespace r2k_core
{

PoseDataLoader::PoseDataLoader(
  const std::string & name, const Header & header, const std::string & child_frame_id)
: DataLoader<PoseDataLoader::ReturnType>(name), header_(header), child_frame_id_(child_frame_id)
{
}

PoseDataLoader::PoseDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header,
  const std::string & child_frame_id)
: DataLoader<PoseDataLoader::ReturnType>(name, logger),
  header_(header),
  child_frame_id_(child_frame_id)
{
}

bool PoseDataLoader::setup_internal(
  const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  if (const auto poses_opt = extract_poses_from_file(load_path); !poses_opt.has_value()) {
    RCLCPP_WARN(
      logger_, "%s pose data loader setup failed. Could not extract poses from file at %s",
      name().c_str(), load_path.string().c_str());
  } else if (const auto number_poses = poses_opt.value().size();
             number_poses != timestamps.size()) {
    RCLCPP_WARN(
      logger_,
      "%s pose data loader setup failed. Number of timestamps do not match number of poses "
      "extracted from file (%zu vs %zu)",
      name().c_str(), timestamps.size(), number_poses);
  } else {
    timestamps_ = timestamps;
    poses_ = poses_opt.value();
    ready_ = true;
    RCLCPP_INFO(
      logger_,
      "%s pose data loader setup successfully. Number of poses with corresponding timestamps: %zu",
      name().c_str(), data_size());
  }
  return ready();
}

bool PoseDataLoader::prepare_data_internal([[maybe_unused]] const std::size_t idx) { return true; }

[[nodiscard]] PoseDataLoader::OptionalReturnType PoseDataLoader::get_data_internal(
  const std::size_t idx)
{
  PoseDataLoader::ReturnType output;
  output.header = header_;
  output.header.stamp = timestamps_.at(idx);
  output.child_frame_id = child_frame_id_;
  output.transform = poses_.at(idx);
  return output;
}

}  // namespace r2k_core
