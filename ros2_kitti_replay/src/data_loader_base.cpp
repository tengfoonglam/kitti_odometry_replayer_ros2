#include "ros2_kitti_replay/data_loader_base.hpp"

namespace r2k_replay
{

DataLoaderBase::DataLoaderBase(const std::string & name)
: name_(name), logger_(rclcpp::get_logger(name))
{
}

DataLoaderBase::DataLoaderBase(const std::string & name, rclcpp::Logger logger)
: name_(name), logger_(logger)
{
}

bool DataLoaderBase::setup(const Timestamps & timestamps, const std::filesystem::path & load_path)
{
  if (!ready()) {
    RCLCPP_WARN(
      logger_, "%s data loader already ready and setup, ignoring setup() call", name().c_str());
    return false;
  }
  return setup_internal(timestamps, load_path);
}

bool DataLoaderBase::prepare_data(const std::size_t idx)
{
  if (!can_process_data(idx, __func__)) {
    return false;
  }
  return prepare_data_internal(idx);
}

DataLoaderBase::~DataLoaderBase() {}

bool DataLoaderBase::can_process_data(const std::size_t idx, const std::string & call_name)
{
  if (ready() && idx < timestamps_.size()) {
    return true;
  } else {
    RCLCPP_WARN(
      logger_,
      "%s data loader could not process %s with the requested index %lu. Debug info - ready: %s"
      ", number packets: %lu",
      name().c_str(), call_name.c_str(), idx, ready() ? "true" : "false", number_packets());
    return false;
  }
}

}  // namespace r2k_replay
