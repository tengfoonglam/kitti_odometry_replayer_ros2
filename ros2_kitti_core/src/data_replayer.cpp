#include "ros2_kitti_core/data_replayer.hpp"

#include <chrono>
#include <functional>
#include <type_traits>

namespace r2k_core
{

DataReplayer::SetTimeRangeRequest::SetTimeRangeRequest(
  const Timestamp & start_time_in, const Timestamp & target_time_in)
: start_time(start_time_in), target_time(target_time_in)
{
}

DataReplayer::StepRequest::StepRequest(std::size_t number_steps_in, float replay_speed_in)
: number_steps(number_steps_in), replay_speed(replay_speed_in)
{
}

DataReplayer::DataReplayer(const std::string & name, const Timestamps & timestamps)
: DataReplayer(name, timestamps, rclcpp::get_logger(name))
{
}

bool operator==(const DataReplayer::ReplayerState & lhs, const DataReplayer::ReplayerState & rhs)
{
  return (lhs.playing == rhs.playing) && (lhs.replay_speed == rhs.replay_speed) &&
         (lhs.start_time == rhs.start_time) && (lhs.current_time == rhs.current_time) &&
         (lhs.target_time == rhs.target_time) && (lhs.end_time == rhs.end_time) &&
         (lhs.next_idx == rhs.next_idx) && (lhs.target_idx == rhs.target_idx) &&
         (lhs.end_idx == rhs.end_idx) && (lhs.start_idx == rhs.start_idx);
}

DataReplayer::DataReplayer(
  const std::string & name, const Timestamps & timestamps, rclcpp::Logger logger)
: name_(name), timestamps_(timestamps), logger_(logger)
{
  if (timestamps.empty()) {
    RCLCPP_ERROR(
      logger_,
      "Replayer %s initialized with an empty timeline, this will lead to unexpected behavior",
      name_.c_str());
    return;
  }

  modify_state([&timestamps = std::as_const(timestamps_)](auto & replayer_state) {
    replayer_state.start_idx = 0;
    replayer_state.start_time = timestamps.front();
    replayer_state.current_time = timestamps.front();
    replayer_state.target_time = timestamps.back();
    replayer_state.end_time = timestamps.back();
    replayer_state.end_idx = timestamps.size();
    replayer_state.next_idx = 0;
    replayer_state.target_idx = timestamps.size() - 1;
  });
}

bool DataReplayer::is_playing() const
{
  return with_lock(state_mutex_, [this] [[nodiscard]] () { return state_.playing; });
}

bool DataReplayer::add_play_data_interface(
  std::shared_ptr<PlayDataInterfaceBase> play_data_interface_ptr)
{
  const auto interface_name = play_data_interface_ptr->name();
  if (is_playing()) {
    RCLCPP_WARN(
      logger_,
      "Replayer %s could not add play data interface %s because it is in the process of playing",
      name_.c_str(), interface_name.c_str());
    return false;
  }
  {
    std::scoped_lock lock(interface_mutex_);
    play_data_interface_ptrs_.push_back(play_data_interface_ptr);
  }
  RCLCPP_INFO(
    logger_, "Successfully added interface %s to replayer %s", interface_name.c_str(),
    name_.c_str());
  return true;
}

bool DataReplayer::set_state_change_cb(StateChangeCallback && state_change_cb)
{
  if (is_playing()) {
    RCLCPP_WARN(
      logger_,
      "Replayer %s could not set state change callback because it is in the process of "
      "playing",
      name_.c_str());
    return false;
  }

  with_lock(cb_mutex_, [this, &state_change_cb = std::as_const(state_change_cb)]() {
    if (state_change_cb_) {
      RCLCPP_WARN(
        logger_, "Replayer %s replacing existing state change callback with new one",
        name_.c_str());
    }
    state_change_cb_ = std::move(state_change_cb);
  });

  return true;
}

bool DataReplayer::set_time_range(const SetTimeRangeRequest & set_time_range_request)
{
  // Lock state till the end of function execution, prevent Time-of-check to time-of-use (TOCTOU)
  // bug
  std::scoped_lock lock(state_mutex_);

  // Return if already playing
  if (state_.playing) {
    RCLCPP_WARN(logger_, "Cannot set time range when replayer %s is playing", name_.c_str());
    return false;
  }

  // Return if invalid play request
  const auto index_range_opt = process_set_time_range_request(set_time_range_request, timestamps_);
  if (!index_range_opt.has_value()) {
    RCLCPP_WARN(
      logger_, "Replayer %s cannot process invalid play request from %fs to %fs", name_.c_str(),
      set_time_range_request.start_time.seconds(), set_time_range_request.target_time.seconds());
    return false;
  }

  // Modify next and target times
  const auto [start_index, target_index] = index_range_opt.value();
  modify_state_no_lock([start_index, target_index, this](auto & replayer_state) {
    replayer_state.current_time = timestamps_.at(start_index);
    replayer_state.next_idx = start_index;
    replayer_state.target_time = timestamps_.at(target_index);
    replayer_state.target_idx = target_index;

    RCLCPP_INFO(
      logger_, "Replayer set to play time range between from %fs to %fs",
      replayer_state.current_time.seconds(), replayer_state.target_time.seconds());
  });

  return true;
}

bool DataReplayer::step(const StepRequest & step_request)
{
  // Return if invalid step request
  const auto state = get_replayer_state();
  const auto index_range_opt = process_step_request(step_request, state.next_idx, state.end_idx);

  if (!index_range_opt.has_value()) {
    RCLCPP_WARN(logger_, "Replayer %s cannot process invalid step request", name_.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_, "Replayer %s playing %zu steps at x%f speed", name_.c_str(), step_request.number_steps,
    step_request.replay_speed);

  return play_index_range(index_range_opt.value(), step_request.replay_speed);
}

bool DataReplayer::play(float replay_speed)
{
  const auto state = get_replayer_state();
  const bool resumable = state.next_idx < state.end_idx;
  if (resumable) {
    const auto target_idx =
      state.next_idx <= state.target_idx ? state.target_idx : state.end_idx - 1;
    return play_index_range({state.next_idx, target_idx}, replay_speed);
  } else {
    RCLCPP_WARN(
      logger_,
      "Replayer %s cannot play because it has either reached the target time or the end of the "
      "timeline",
      name_.c_str());
    return false;
  }
}

bool DataReplayer::play_index_range(const IndexRange & index_range, float replay_speed)
{
  // Lock state till the end of function execution, prevent Time-of-check to time-of-use (TOCTOU)
  // bug
  std::scoped_lock lock(state_mutex_);

  // Return if already playing
  if (state_.playing) {
    RCLCPP_WARN(logger_, "Replayer %s is already playing", name_.c_str());
    return false;
  }

  // Update state and start thread
  const auto [start_index, target_index] = index_range;

  if (replay_speed <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Replayer %s requested to play at a zero/negative replay speed of %f. Will play as fast as "
      "possible.",
      name_.c_str(), replay_speed);
  }

  // Update state
  modify_state_no_lock([start_index, target_index, replay_speed, this](auto & replayer_state) {
    replayer_state.playing = true;
    replayer_state.current_time = timestamps_.at(start_index);
    replayer_state.next_idx = start_index;
    replayer_state.target_time = timestamps_.at(target_index);
    replayer_state.target_idx = target_index;
    replayer_state.replay_speed = std::max(0.0f, replay_speed);
  });

  // Ensure play thread has been stopped. state_.playing == true should guarantee that previous
  // play_loop has completed
  stop_play_thread();

  play_thread_shutdown_flag_ = false;
  play_thread_ptr_ = std::make_unique<std::thread>(&DataReplayer::play_loop, this);

  RCLCPP_INFO(
    logger_, "Replayer %s starting to play from %fs to %fs at x%f speed", name_.c_str(),
    state_.current_time.seconds(),
    timestamps_.at(std::min<std::size_t>(state_.target_idx, state_.end_idx - 1)).seconds(),
    state_.replay_speed);

  return true;
}

std::size_t DataReplayer::get_next_index() const
{
  return with_lock(state_mutex_, [&state_ = std::as_const(state_)]() { return state_.next_idx; });
}

void DataReplayer::prepare_data(std::size_t index_to_prep)
{
  std::scoped_lock lock(interface_mutex_);

  std::for_each(
    std::cbegin(play_data_interface_ptrs_), std::cend(play_data_interface_ptrs_),
    [this, index_to_prep](const auto & interface_ptr) {
      const auto prepare_success = interface_ptr->prepare(index_to_prep);
      if (!prepare_success) {
        RCLCPP_WARN(
          logger_, "Failed to prepare data for %s, index %ld", interface_ptr->name().c_str(),
          index_to_prep);
      }
    });
}

void DataReplayer::play_data(std::size_t index_to_play)
{
  std::scoped_lock lock(interface_mutex_);

  std::for_each(
    std::cbegin(play_data_interface_ptrs_), std::cend(play_data_interface_ptrs_),
    [this, index_to_play](const auto & interface_ptr) {
      const auto play_success = interface_ptr->play(index_to_play);
      if (!play_success) {
        RCLCPP_WARN(
          logger_, "Failed to play data for %s, index %ld", interface_ptr->name().c_str(),
          index_to_play);
      }
    });
}

void DataReplayer::play_loop()
{
  RCLCPP_INFO(logger_, "Replayer %s's play loop started", name_.c_str());

  // Prepare data of first frame before starting loop
  prepare_data(get_next_index());

  const auto replay_speed =
    with_lock(state_mutex_, [this] [[nodiscard]] () { return state_.replay_speed; });

  while (!play_thread_shutdown_flag_) {
    // Start timer
    const auto iteration_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // Get current index
    const auto current_index = get_next_index();
    const auto next_index = current_index + 1;

    // Play
    const auto play_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    play_data(current_index);
    const auto play_end_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const auto play_duration = play_end_time - play_start_time;
    RCLCPP_DEBUG(logger_, "Time taken to play data: %fs", play_duration.seconds());

    // Update state
    modify_state([this, &timestamps = std::as_const(timestamps_)](auto & replayer_state) {
      replayer_state.next_idx++;
      replayer_state.current_time = replayer_state.next_idx < replayer_state.end_idx
                                      ? timestamps.at(replayer_state.next_idx)
                                      : replayer_state.end_time;

      RCLCPP_DEBUG(logger_, "Current time: %fs", replayer_state.current_time.seconds());
    });

    // If play has been completed, break from loop
    const auto play_complete = with_lock(state_mutex_, [&] [[nodiscard]] () {
      return (state_.next_idx > state_.target_idx) || (state_.next_idx >= state_.end_idx);
    });
    if (play_complete) {
      break;
    }

    // Prepare data for next index
    const auto prep_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    prepare_data(next_index);
    const auto prep_end_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const auto prep_duration = prep_end_time - prep_start_time;
    RCLCPP_DEBUG(logger_, "Time taken for data preparation: %fs", prep_duration.seconds());

    // Compute time to wait till next iteration
    const auto iteration_end_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const auto iteration_duration = iteration_end_time - iteration_start_time;
    const auto duration_between_frames = timestamps_.at(next_index) - timestamps_.at(current_index);
    const auto duration_till_next_play =
      replay_speed == 0.0 ? rclcpp::Duration(0, 0)
                          : (duration_between_frames * (1.0 / replay_speed) - iteration_duration);

    // Break if flag has been set
    if (play_thread_shutdown_flag_) {
      break;
    }

    // If duration till wait is negative, give a warning and immediately continue to next iteration
    if (duration_till_next_play < rclcpp::Duration(0, 0)) {
      RCLCPP_WARN(
        logger_, "Replayer %s behind schedule by %f seconds", name_.c_str(),
        -1.0 * duration_till_next_play.seconds());
      continue;
    }

    // Else wait
    std::unique_lock lock(flag_mutex_);
    play_thread_cv_.wait_for(
      lock, duration_till_next_play.to_chrono<std::chrono::nanoseconds>(),
      [&play_thread_shutdown_flag_ = std::as_const(play_thread_shutdown_flag_)] [[nodiscard]] () {
        return play_thread_shutdown_flag_.load();
      });
  }

  // Set final state
  modify_state([](auto & replayer_state) { replayer_state.playing = false; });
  RCLCPP_INFO(logger_, "Replayer %s's play loop ended", name_.c_str());
}

bool DataReplayer::pause()
{
  if (!is_playing()) {
    RCLCPP_WARN(logger_, "Replayer %s is not playing, already paused", name_.c_str());
    return true;
  }

  stop_play_thread();

  RCLCPP_INFO(logger_, "Replayer %s paused", name_.c_str());

  return true;
}

bool DataReplayer::stop()
{
  if (is_playing()) {
    stop_play_thread();
  }

  const auto reset_success = reset();
  RCLCPP_INFO(
    logger_, "Replayer %s stopped %s", name_.c_str(),
    reset_success ? "successfully" : "unsuccessfully");

  return reset_success;
}

bool DataReplayer::reset()
{
  if (is_playing()) {
    RCLCPP_WARN(logger_, "Cannot reset replayer %s when it is playing", name_.c_str());
    return false;
  }

  modify_state([&timestamps = std::as_const(timestamps_)](auto & replayer_state) {
    replayer_state.next_idx = 0;
    replayer_state.current_time = replayer_state.start_time;
  });

  RCLCPP_INFO(logger_, "Replayer %s has been reset", name_.c_str());

  return true;
}

void DataReplayer::stop_play_thread()
{
  std::scoped_lock lock(thread_mutex_);
  if (play_thread_ptr_ && play_thread_ptr_->joinable()) {
    play_thread_shutdown_flag_ = true;
    play_thread_cv_.notify_all();
    play_thread_ptr_->join();
    play_thread_shutdown_flag_ = false;
  }
}

DataReplayer::~DataReplayer() { stop_play_thread(); }

DataReplayer::ReplayerState DataReplayer::get_replayer_state() const
{
  return with_lock(state_mutex_, [this] [[nodiscard]] () { return state_; });
}

void DataReplayer::modify_state_no_lock(const StateModificationCallback & modify_cb)
{
  std::scoped_lock lock(cb_mutex_);
  modify_cb(state_);
  if (state_change_cb_) {
    state_change_cb_(state_);
  }
}

void DataReplayer::modify_state(const StateModificationCallback & modify_cb)
{
  std::scoped_lock lock(state_mutex_);
  modify_state_no_lock(modify_cb);
};

DataReplayer::IndexRangeOpt DataReplayer::process_set_time_range_request(
  const SetTimeRangeRequest & set_time_range_request, const Timestamps & timestamps)
{
  // Return immediately if timestamp is empty
  if (timestamps.empty()) {
    return std::nullopt;
  }

  // Aliases
  const auto & start_time = set_time_range_request.start_time;
  const auto & target_time = set_time_range_request.target_time;

  // Return if start_time is after last timestamp or target_time is before first timestamp
  if (start_time > timestamps.back() || target_time < timestamps.front()) {
    return std::nullopt;
  }

  // Find starting index
  std::size_t start_index{0};
  for (auto i = start_index; i < timestamps.size(); i++) {
    const auto & timestamp = timestamps.at(i);
    start_index = i;
    if (timestamp >= start_time) {
      break;
    }
  }

  // Find target index
  std::size_t target_index{timestamps.size()};
  for (auto i = target_index; i-- > 0;) {
    const auto & timestamp = timestamps.at(i);
    target_index = i;
    if (timestamp <= target_time) {
      break;
    }
  }

  // Return result
  return (target_index >= start_index) ? std::optional(std::make_tuple(start_index, target_index))
                                       : std::nullopt;
}

DataReplayer::IndexRangeOpt DataReplayer::process_step_request(
  const StepRequest & step_request, std::size_t next_idx, std::size_t end_idx)
{
  // Invalid if step size is zero
  if (step_request.number_steps < 1) {
    return std::nullopt;
  }

  // Invalid if replayer is at the end of the timeline
  if (next_idx >= end_idx) {
    return std::nullopt;
  }

  // Compute target index and return
  const auto target_idx = std::min(next_idx + step_request.number_steps - 1, end_idx - 1);
  return std::make_pair(next_idx, target_idx);
}

}  // namespace r2k_core
