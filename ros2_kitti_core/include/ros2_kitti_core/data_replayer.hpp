#ifndef ROS2_KITTI_CORE__DATA_REPLAYER_HPP_
#define ROS2_KITTI_CORE__DATA_REPLAYER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/play_data_interface_base.hpp"

namespace r2k_core
{

class DataReplayer
{
public:
  struct ReplayerState
  {
    bool playing{false};
    float replay_speed{1.0f};
    Timestamp start_time;
    Timestamp current_time;
    Timestamp target_time;
    Timestamp end_time;
    std::size_t start_idx{0};
    std::size_t next_idx{0};
    std::size_t target_idx{0};
    std::size_t end_idx{0};

    friend bool operator==(const ReplayerState & lhs, const ReplayerState & rhs);
  };

  struct TimeRange
  {
    Timestamp start_time;
    Timestamp end_time;
    TimeRange(
      const Timestamp & start_time_in = Timestamp{0, 0},
      const Timestamp & end_time_in = Timestamp{0, 0});
  };

  struct StepRequest
  {
    std::size_t number_steps{0};
    float replay_speed{1.0f};
    explicit StepRequest(std::size_t number_steps_in, float replay_speed_in = 1.0f);
  };

  using StateChangeCallback = std::function<void(const ReplayerState &)>;
  using StateModificationCallback = std::function<void(ReplayerState &)>;
  using IndexRange = std::tuple<std::size_t, std::size_t>;
  using IndexRangeOpt = std::optional<IndexRange>;

  DataReplayer(
    const std::string & name, const Timestamps & timestamps,
    rclcpp::Logger logger = rclcpp::get_logger("replayer"),
    const TimeRange & time_range = TimeRange());

  DataReplayer(const DataReplayer & other) = delete;
  DataReplayer & operator=(const DataReplayer & other) = delete;

  [[nodiscard]] bool is_playing() const;

  bool add_play_data_interface(std::shared_ptr<PlayDataInterfaceBase> play_data_interface_ptr);

  bool set_state_change_cb(StateChangeCallback && state_change_cb);

  [[nodiscard]] ReplayerState get_replayer_state() const;

  bool play(float replay_speed = 1.0f);

  bool set_next_play_time_range(const TimeRange & next_play_time_range);

  bool step(const StepRequest & step_request);

  bool pause();

  bool stop();

  bool reset();

  ~DataReplayer();

  [[nodiscard]] static IndexRangeOpt get_index_range_from_time_range(
    const TimeRange & next_play_time_range, std::size_t start_idx, std::size_t end_idx,
    const Timestamps & timestamps);

  [[nodiscard]] static IndexRangeOpt process_step_request(
    const StepRequest & step_request, std::size_t next_idx, std::size_t end_idx);

private:
  const std::string name_;
  const Timestamps timestamps_;  // Note: const member variable ok since this class cannot be
                                 // moved/copied due to mutex anyway
  rclcpp::Logger logger_;

  mutable std::mutex state_mutex_;
  ReplayerState state_;

  mutable std::mutex thread_mutex_;
  std::shared_ptr<std::thread> play_thread_ptr_;

  mutable std::mutex flag_mutex_;
  std::atomic_bool play_thread_shutdown_flag_{false};
  std::condition_variable play_thread_cv_;

  mutable std::mutex cb_mutex_;
  StateChangeCallback state_change_cb_;

  mutable std::mutex interface_mutex_;
  std::vector<std::shared_ptr<PlayDataInterfaceBase>> play_data_interface_ptrs_;

  template <typename Callable>
  static inline auto with_lock(std::mutex & mutex, Callable callable) -> decltype(callable())
  {
    std::scoped_lock lock(mutex);
    return callable();
  }

  void modify_state_no_lock(const StateModificationCallback & modify_cb);

  void modify_state(const StateModificationCallback & modify_cb);

  void play_loop();

  void stop_play_thread();

  [[nodiscard]] std::size_t get_next_index() const;

  void prepare_data(std::size_t index);

  void play_data(std::size_t index);

  bool play_index_range(const IndexRange & index_range, float replay_speed);
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__DATA_REPLAYER_HPP_
