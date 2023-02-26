#ifndef ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_
#define ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros2_kitti_replay/data_loader.hpp"
#include "ros2_kitti_replay/play_data_callback_base.hpp"

namespace r2k_replay
{

class DataReplayer
{
public:
  struct ReplayerState
  {
    bool playing;
    float replay_speed;
    Timestamp start_time;
    Timestamp current_time;
    Timestamp final_time;
    std::size_t next_idx;
    std::size_t data_size;
  };

  struct PlayRequest
  {
    float replay_speed;
    Timestamp start_time;
    Timestamp final_time;

    [[nodiscard]] bool is_valid(const Timestamps & timeline) const;
  };

  using StateChangeCallback = std::function<bool(const ReplayerState &)>;

  explicit DataReplayer(const std::string & name, const Timestamps & timestamps);
  DataReplayer(const std::string & name, const Timestamps & timestamps, rclcpp::Logger logger);

  DataReplayer(const DataReplayer & other) = delete;
  DataReplayer & operator=(const DataReplayer & other) = delete;

  [[nodiscard]] bool is_playing() const;

  bool add_play_data_cb(std::unique_ptr<PlayDataCallbackBase> play_data_cb_ptr);

  bool set_state_change_cb(StateChangeCallback && state_change_cb);

  [[nodiscard]] ReplayerState getReplayerState() const;

  bool play(const PlayRequest & play_request);

  bool step(const std::size_t number_steps);

  bool pause();

  bool stop();

  bool reset();

  ~DataReplayer();

private:
  const std::string name_;
  const Timestamps timestamps_;  // Note: const member variable ok since this class cannot be
                                 // moved/copied due to mutex anyway

  mutable std::mutex state_mutex_;
  ReplayerState state_;

  mutable std::mutex logger_mutex_;
  rclcpp::Logger logger_;

  mutable std::mutex thread_mutex_;
  std::unique_ptr<std::thread> play_thread_ptr_;

  mutable std::mutex cb_mutex_;
  std::vector<std::unique_ptr<PlayDataCallbackBase>> play_data_cb_ptrs_;
  StateChangeCallback state_change_cb_;

  template <typename Callable>
  inline auto with_lock(std::mutex & mutex, Callable callable) -> decltype(callable())
  {
    std::scoped_lock lock(mutex);
    return callable();
  }
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_
