#include "ros2_kitti_replay/clock_data_loader.hpp"

namespace r2k_replay
{

ClockDataLoader::ClockDataLoader(const std::string & name) : DataLoader<ClockDataLoader::Type>(name)
{
}

ClockDataLoader::ClockDataLoader(const std::string & name, rclcpp::Logger logger)
: DataLoader<ClockDataLoader::Type>(name, logger)
{
}

bool ClockDataLoader::setup_internal(
  const Timestamps & timestamps, [[maybe_unused]] const std::filesystem::path & load_path)
{
  timestamps_ = timestamps;
  ready_ = true;
  return ready();
}

[[nodiscard]] ClockDataLoader::OptionalType ClockDataLoader::get_data_internal(
  const std::size_t idx)
{
  ClockDataLoader::Type clock_msg;
  clock_msg.set__clock(timestamps_.at(idx));
  return OptionalType(clock_msg);
}

bool ClockDataLoader::prepare_data_internal([[maybe_unused]] const std::size_t idx) { return true; }

}  // namespace r2k_replay
