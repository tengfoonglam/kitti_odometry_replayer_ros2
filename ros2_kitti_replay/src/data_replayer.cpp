#include "ros2_kitti_replay/data_replayer.hpp"

namespace r2k_replay
{

DataReplayer::DataReplayer(const std::string & name, const Timestamps & timestamps)
: DataReplayer(name, timestamps, rclcpp::get_logger(name))
{
}

DataReplayer::DataReplayer(
  const std::string & name, const Timestamps & timestamps, rclcpp::Logger logger)
: name_(name), timestamps_(timestamps), logger_(logger)
{
  modify_state([&timestamps = std::as_const(timestamps_)](auto & replayer_state) {
    if (timestamps.empty()) {
      return;
    }
    replayer_state.start_time = timestamps.front();
    replayer_state.final_time = timestamps.back();
    replayer_state.data_size = timestamps.size();
  });
}

[[nodiscard]] bool DataReplayer::is_playing() const
{
  return with_lock(state_mutex_, [this] [[nodiscard]] () { return state_.playing; });
}

bool DataReplayer::add_play_data_cb(std::unique_ptr<PlayDataCallbackBase> play_data_cb_ptr)
{
  if (is_playing()) {
    with_lock(logger_mutex_, [this, cb_name = play_data_cb_ptr->name()]() {
      RCLCPP_WARN(
        logger_,
        "Replayer %s could not add play data callback %s because it is in the process of playing",
        name_.c_str(), cb_name.c_str());
    });
    return false;
  }
  {
    std::scoped_lock lock(cb_mutex_);
    play_data_cb_ptrs_.push_back(std::move(play_data_cb_ptr));
  }
  return true;
}

bool DataReplayer::set_state_change_cb(const StateChangeCallback & state_change_cb)
{
  if (is_playing()) {
    with_lock(logger_mutex_, [this]() {
      RCLCPP_WARN(
        logger_,
        "Replayer %s could not set state change callback because it is in the process of "
        "playing",
        name_.c_str());
    });
    return false;
  }

  with_lock(cb_mutex_, [this, &state_change_cb = std::as_const(state_change_cb)]() {
    if (state_change_cb_) {
      with_lock(logger_mutex_, [this]() {
        RCLCPP_WARN(
          logger_, "Replayer %s replacing existing state change callback with new one",
          name_.c_str());
      });
    }
    state_change_cb_ = state_change_cb;
  });

  return true;
}

bool DataReplayer::play(const PlayRequest & play_request)
{
  if (is_playing()) {
    with_lock(logger_mutex_, [this]() {
      RCLCPP_WARN(logger_, "Replayer %s is already playing", name_.c_str());
    });
    return false;
  }

  return true;
}

bool DataReplayer::step(const StepRequest & step_request) { return false; }

bool DataReplayer::pause() { return false; }

bool DataReplayer::stop() { return false; }

bool DataReplayer::reset() { return false; }

DataReplayer::~DataReplayer()
{
  std::scoped_lock lock(thread_mutex_);
  if (play_thread_ptr_ && play_thread_ptr_->joinable()) {
    play_thread_ptr_->join();
  }
}

[[nodiscard]] DataReplayer::ReplayerState DataReplayer::getReplayerState() const
{
  return with_lock(state_mutex_, [this] [[nodiscard]] () { return state_; });
};

void DataReplayer::modify_state(const StateModificationCallback & modify_cb)
{
  std::scoped_lock lock(state_mutex_, cb_mutex_);
  modify_cb(state_);
  state_change_cb_(state_);
};

std::optional<std::pair<std::size_t, std::size_t>> DataReplayer::compute_index_range_to_play(
  const PlayRequest & play_request, const Timestamps & timestamps)
{
  // Return immediately if timestamp is empty
  if (timestamps.empty()) {
    return std::nullopt;
  }

  // Find starting index
  std::size_t start_index{0};
  const auto & start_time = play_request.start_time;
  for (; start_index < timestamps.size(); start_index++) {
    const auto timestamp = timestamps.at(start_index);
    if (timestamp >= start_time) {
      break;
    }
  }

  // Find target index
  std::size_t target_index{timestamps.size()};
  const auto & target_time = play_request.target_time;
  for (; target_index-- > 0;) {
    const auto timestamp = timestamps.at(target_index);
    if (timestamp <= target_time) {
      break;
    }
  }

  // Return result
  return (target_index >= start_index) ? std::optional(std::make_pair(start_index, target_index))
                                       : std::nullopt;
}

}  // namespace r2k_replay
