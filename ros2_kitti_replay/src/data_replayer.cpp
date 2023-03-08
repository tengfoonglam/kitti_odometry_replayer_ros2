#include "ros2_kitti_replay/data_replayer.hpp"

#include <chrono>
#include <functional>
#include <type_traits>

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
  // Return if invalid play request
  const auto index_range_opt = compute_index_range_to_play(play_request, timestamps_);
  if (!index_range_opt.has_value()) {
    with_lock(logger_mutex_, [this, &play_request = std::as_const(play_request)]() {
      RCLCPP_WARN(
        logger_, "Replayer %s cannot process invalid play request from %fs to %fs", name_.c_str(),
        play_request.start_time.seconds(), play_request.target_time.seconds());
    });
    return false;
  }

  // Lock state till the end of function execution, prevent Time-of-check to time-of-use (TOCTOU)
  // bug
  std::scoped_lock lock(state_mutex_);

  // Return if already playing
  if (state_.playing) {
    with_lock(logger_mutex_, [this]() {
      RCLCPP_WARN(logger_, "Replayer %s is already playing", name_.c_str());
    });
    return false;
  }

  // Update state and start thread
  const auto [start_index, target_index] = index_range_opt.value();

  if (play_request.replay_speed <= 0.0) {
    with_lock(logger_mutex_, [this, &play_request = std::as_const(play_request)]() {
      RCLCPP_WARN(
        logger_,
        "Replayer %s requested to play at a zero/negative replay speed of %f. Will play as fast as "
        "possible.",
        name_.c_str(), play_request.replay_speed);
    });
  }

  state_.playing = true;
  state_.current_time = start_index > 0 ? timestamps_.at(start_index - 1) : Timestamp();
  state_.next_idx = start_index;
  state_.target_idx = target_index;
  state_.replay_speed = std::max(
    static_cast<std::decay<decltype(play_request.replay_speed)>::type>(0.0),
    play_request.replay_speed);

  // Ensure play thread has been stopped. state_.playing == true should guarantee that previous
  // play_loop has completed
  stop_play_thread();

  play_thread_shutdown_flag_ = false;
  play_thread_ptr_ = std::make_unique<std::thread>(&DataReplayer::play_loop, this);

  return true;
}

[[nodiscard]] size_t DataReplayer::get_current_index() const
{
  return with_lock(state_mutex_, [&]() { return state_.next_idx; });
}

void DataReplayer::prepare_data(const size_t index_to_prep)
{
  with_lock(cb_mutex_, [&, index_to_prep]() {
    std::for_each(
      std::cbegin(play_data_cb_ptrs_), std::cend(play_data_cb_ptrs_),
      [index_to_prep](const auto & cb_ptr) { cb_ptr->prepare(index_to_prep); });
  });
}

void DataReplayer::play_data(const size_t index_to_play)
{
  with_lock(cb_mutex_, [&, index_to_play]() {
    std::for_each(
      std::cbegin(play_data_cb_ptrs_), std::cend(play_data_cb_ptrs_),
      [index_to_play](const auto & cb_ptr) { cb_ptr->play(index_to_play); });
  });
}

void DataReplayer::play_loop()
{
  // Prepare data of first frame before starting loop
  prepare_data(get_current_index());

  while (!play_thread_shutdown_flag_) {
    // Start timer
    const auto iteration_start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // Get current index
    const auto current_index = get_current_index();
    const auto next_index = current_index + 1;

    // Play
    play_data(current_index);

    // Update state
    with_lock(state_mutex_, [&]() {
      state_.next_idx++;
      state_.current_time =
        state_.next_idx < state_.data_size ? timestamps_.at(state_.next_idx) : state_.final_time;
    });

    // If play has been completed, break from loop
    const auto play_complete = with_lock(state_mutex_, [&] [[nodiscard]] () {
      return (state_.next_idx > state_.target_idx) || (state_.next_idx >= state_.data_size);
    });
    if (play_complete) {
      break;
    }

    // Prepare data for next index
    prepare_data(next_index);

    // Compute time to wait till next iteration
    const auto iteration_end_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const auto iteration_duration = iteration_end_time - iteration_start_time;
    const auto duration_between_frames = timestamps_.at(next_index) - timestamps_.at(current_index);
    const auto duration_till_next_play = duration_between_frames - iteration_duration;

    // Break if flag has been set
    if (play_thread_shutdown_flag_) {
      break;
    }

    // If duration till wait is negative, give a warning and immediately continue to next iteration
    if (duration_till_next_play < decltype(duration_till_next_play){0, 0}) {
      with_lock(logger_mutex_, [this, &duration_till_next_play]() {
        RCLCPP_WARN(
          logger_, "Replayer %s behind schedule by %f seconds", name_.c_str(),
          duration_till_next_play.seconds());
      });
      continue;
    }

    // Else wait
    std::unique_lock lock(state_mutex_);
    play_thread_cv_.wait_for(
      lock, duration_till_next_play.to_chrono<std::chrono::nanoseconds>(),
      [&play_thread_shutdown_flag_ = std::as_const(play_thread_shutdown_flag_)] [[nodiscard]] () {
        return play_thread_shutdown_flag_.load();
      });
  }

  // Set final state
  with_lock(state_mutex_, [&]() { state_.playing = false; });
}

bool DataReplayer::step([[maybe_unused]] const StepRequest & step_request) { return false; }

bool DataReplayer::pause() { return false; }

bool DataReplayer::stop() { return false; }

bool DataReplayer::reset() { return false; }

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

std::optional<DataReplayer::IndexRange> DataReplayer::compute_index_range_to_play(
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
  return (target_index >= start_index) ? std::optional(std::make_tuple(start_index, target_index))
                                       : std::nullopt;
}

}  // namespace r2k_replay
