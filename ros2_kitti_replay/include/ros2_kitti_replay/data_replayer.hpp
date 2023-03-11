#ifndef ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_
#define ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
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
    float replay_speed{1.0f};
    Timestamp start_time;
    Timestamp current_time;
    Timestamp final_time;
    std::size_t next_idx{};
    std::size_t target_idx{};
    std::size_t data_size{};
  };

  struct PlayRequest
  {
    Timestamp start_time;
    Timestamp target_time;
    float replay_speed{1.0f};
    PlayRequest(
      const Timestamp & start_time_in, const Timestamp & target_timein,
      const float replay_speed_in = 1.0f);
  };

  struct StepRequest
  {
    float replay_speed{1.0f};
    std::size_t number_steps{};
  };

  using StateChangeCallback = std::function<void(const ReplayerState &)>;
  using StateModificationCallback = std::function<void(ReplayerState &)>;
  using IndexRange = std::tuple<std::size_t, std::size_t>;
  using IndexRangeOpt = std::optional<IndexRange>;

  explicit DataReplayer(const std::string & name, const Timestamps & timestamps);
  DataReplayer(const std::string & name, const Timestamps & timestamps, rclcpp::Logger logger);

  DataReplayer(const DataReplayer & other) = delete;
  DataReplayer & operator=(const DataReplayer & other) = delete;

  [[nodiscard]] bool is_playing() const;

  bool add_play_data_cb(std::unique_ptr<PlayDataCallbackBase> play_data_cb_ptr);

  bool set_state_change_cb(const StateChangeCallback & state_change_cb);

  [[nodiscard]] ReplayerState get_replayer_state() const;

  bool play(const PlayRequest & play_request);

  bool step(const StepRequest & step_request);

  bool pause();

  bool stop();

  bool reset();

  ~DataReplayer();

  [[nodiscard]] static IndexRangeOpt process_play_request(
    const PlayRequest & play_request, const Timestamps & timestamps);

  [[nodiscard]] static IndexRangeOpt process_step_request(
    const StepRequest & step_request, const ReplayerState & replayer_state);

private:
  const std::string name_;
  const Timestamps timestamps_;  // Note: const member variable ok since this class cannot be
                                 // moved/copied due to mutex anyway

  mutable std::mutex state_mutex_;
  ReplayerState state_;

  mutable std::mutex logger_mutex_;
  rclcpp::Logger logger_;

  mutable std::mutex thread_mutex_;
  std::atomic_bool play_thread_shutdown_flag_{false};
  std::condition_variable play_thread_cv_;
  std::unique_ptr<std::thread> play_thread_ptr_;

  mutable std::mutex cb_mutex_;
  std::vector<std::unique_ptr<PlayDataCallbackBase>> play_data_cb_ptrs_;
  StateChangeCallback state_change_cb_;

  template <typename Callable>
  static inline auto with_lock(std::mutex & mutex, Callable callable) -> decltype(callable())
  {
    std::scoped_lock lock(mutex);
    return callable();
  }

  void modify_state(const StateModificationCallback & modify_cb);

  void play_loop();

  void stop_play_thread();

  [[nodiscard]] size_t get_current_index() const;

  void prepare_data(const size_t index);

  void play_data(const size_t index);

  bool play_index_range(const IndexRange & index_range, const float replay_speed);
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__DATA_REPLAYER_HPP_
