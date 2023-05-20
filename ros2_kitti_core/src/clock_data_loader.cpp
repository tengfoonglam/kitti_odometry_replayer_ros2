#include "ros2_kitti_core/clock_data_loader.hpp"

namespace r2k_core
{

ClockDataLoader::ClockDataLoader(const std::string & name)
: DataLoader<ClockDataLoader::ReturnType>(name)
{
}

ClockDataLoader::ClockDataLoader(const std::string & name, rclcpp::Logger logger)
: DataLoader<ClockDataLoader::ReturnType>(name, logger)
{
}

bool ClockDataLoader::setup_internal(
  const Timestamps & timestamps, [[maybe_unused]] const std::filesystem::path & load_path)
{
  timestamps_ = timestamps;
  ready_ = true;
  RCLCPP_INFO(
    logger_, "%s Clock data loader setup successful. Number of timestamps: %zu", name().c_str(),
    timestamps_.size());
  return ready();
}

[[nodiscard]] ClockDataLoader::OptionalReturnType ClockDataLoader::get_data_internal(
  const std::size_t idx)
{
  ClockDataLoader::ReturnType clock_msg;
  clock_msg.set__clock(timestamps_.at(idx));
  return clock_msg;
}

bool ClockDataLoader::prepare_data_internal([[maybe_unused]] const std::size_t idx) { return true; }

}  // namespace r2k_core
