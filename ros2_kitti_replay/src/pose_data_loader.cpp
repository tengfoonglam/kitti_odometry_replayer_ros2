#include "ros2_kitti_replay/pose_data_loader.hpp"

#include <algorithm>
namespace r2k_replay
{

PoseDataLoader::PoseDataLoader(
  const std::string & name, const Header & header, const std::string & child_frame_id)
: DataLoader<PoseDataLoader::Type>(name), header_(header), child_frame_id_(child_frame_id)
{
}

PoseDataLoader::PoseDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header,
  const std::string & child_frame_id)
: DataLoader<PoseDataLoader::Type>(name, logger), header_(header), child_frame_id_(child_frame_id)
{
}

bool PoseDataLoader::setup_internal(
  const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  return false;
}

[[nodiscard]] std::size_t PoseDataLoader::data_size() const
{
  return ready() ? std::min(timestamps_.size(), poses_.size()) : std::size_t{0};
}

bool PoseDataLoader::prepare_data_internal(const std::size_t idx) { return true; }

[[nodiscard]] PoseDataLoader::OptionalType PoseDataLoader::get_data_internal(const std::size_t idx)
{
  return std::nullopt;
}

}  // namespace r2k_replay
