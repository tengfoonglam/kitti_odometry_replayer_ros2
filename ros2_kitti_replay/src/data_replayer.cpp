#include "ros2_kitti_replay/data_replayer.hpp"

#include <utility>

namespace r2k_replay
{

[[nodiscard]] bool DataReplayer::PlayRequest::is_valid(const Timestamps & timeline) const
{
  return false;
}

DataReplayer::DataReplayer(const std::string & name, const Timestamps & timestamps)
: DataReplayer(name, timestamps, rclcpp::get_logger(name))
{
}

DataReplayer::DataReplayer(
  const std::string & name, const Timestamps & timestamps, rclcpp::Logger logger)
: name_(name), timestamps_(timestamps), logger_(logger)
{
}

[[nodiscard]] bool DataReplayer::is_playing() const
{
  std::scoped_lock lock(state_mutex_);
  return state_.playing;
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
  with_lock(cb_mutex_, [this, play_data_cb_ptr = std::move(play_data_cb_ptr)]() mutable {
    play_data_cb_ptrs_.push_back(std::move(play_data_cb_ptr));
  });
  return true;
}

bool DataReplayer::set_state_change_cb(StateChangeCallback && state_change_cb)
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

  with_lock(cb_mutex_, [this, &state_change_cb]() {
    if (state_change_cb_) {
      with_lock(logger_mutex_, [this]() {
        RCLCPP_WARN(
          logger_, "Replayer %s replacing existing state change callback with new one",
          name_.c_str());
      });
    }
    std::swap(state_change_cb_, state_change_cb);
  });

  return true;
}

bool DataReplayer::play(const PlayRequest & play_request) { return false; }

bool DataReplayer::step(const std::size_t number_steps) { return false; }

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
  std::scoped_lock lock(state_mutex_);
  return state_;
};

}  // namespace r2k_replay
