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

/**
 * @brief Data replayer that loads and play data given a sequence of timestamps
 *
 */
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

  /**
   * @brief Construct a new Data Replayer object
   *
   * @param name
   * @param timestamps
   * @param logger
   * @param time_range
   */
  DataReplayer(
    const std::string & name, const Timestamps & timestamps,
    rclcpp::Logger logger = rclcpp::get_logger("replayer"),
    const TimeRange & time_range = TimeRange());

  // Rule of Five since destructor was modified
  DataReplayer(const DataReplayer & other) = delete;
  DataReplayer & operator=(const DataReplayer & other) = delete;

  /**
   * @brief Whether replayer is playing
   *
   * @return true - Is playing
   * @return false - Otherwise
   */
  [[nodiscard]] bool is_playing() const;

  /**
   * @brief Add a play data interface to the replayer. Replayer must not be playing
   *
   * @param play_data_interface_ptr - Shared pointer of the interface to be added
   * @return true - Added successfully
   * @return false _ Otherwise
   */
  bool add_play_data_interface(std::shared_ptr<PlayDataInterfaceBase> play_data_interface_ptr);

  /**
   * @brief Set the state change cb object. This will be called whenever the replayer changes state.
   *        Replayer must not be playing
   *
   * @param state_change_cb - Callback to call
   * @return true - Set successful
   * @return false - Otherwise
   */
  bool set_state_change_cb(StateChangeCallback && state_change_cb);

  /**
   * @brief Get the replayer state
   *
   * @return ReplayerState - Current replayer state
   */
  [[nodiscard]] ReplayerState get_replayer_state() const;

  /**
   * @brief Play
   *
   * @param replay_speed
   * @return true - Successful
   * @return false - Otherwise
   */
  bool play(float replay_speed = 1.0f);

  /**
   * @brief Set the time range to play
   *
   * @param next_play_time_range - Time range to play
   * @return true - Set successful
   * @return false - Otherwise
   */
  bool set_next_play_time_range(const TimeRange & next_play_time_range);

  /**
   * @brief Step through a fixed amount of frames
   *
   * @param step_request
   * @return true - Step started successfully
   * @return false - Otherwise
   */
  bool step(const StepRequest & step_request);

  /**
   * @brief Pause the replayer
   *
   * @return true - Paused the replayer successfully or already not playing
   * @return false - Otherwise
   */
  bool pause();

  /**
   * @brief Stop the replayer. Timeline is reset back to the start time
   *
   * @return true - Stopped the replayer successfully
   * @return false - Otherwise
   */
  bool stop();

  /**
   * @brief Reset the replayer
   *
   * @return true - Reset successful
   * @return false - Otherwise
   */
  bool reset();

  /**
   * @brief Destroy the Data Replayer object
   *
   */
  ~DataReplayer();

  /**
   * @brief Get the index range to play given a time range
   *
   * @param next_play_time_range - Time range
   * @param start_idx - Minimum index
   * @param end_idx - Maximum index
   * @param timestamps - Timestamps
   * @return IndexRangeOpt - Index range if a valid one is found
   */
  [[nodiscard]] static IndexRangeOpt get_index_range_from_time_range(
    const TimeRange & next_play_time_range, std::size_t start_idx, std::size_t end_idx,
    const Timestamps & timestamps);

  /**
   * @brief GGet the index range given a step request
   *
   * @param step_request - Step request
   * @param next_idx - Next index to be played
   * @param end_idx - Last index to be played
   * @return IndexRangeOpt - Index range if a valid one is found
   */
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

  /**
   * @brief Lock a given mutex before executing a callable
   *
   * @tparam Callable
   * @param mutex
   * @param callable
   * @return decltype(callable()) - Return value of callable
   */
  template <typename Callable>
  static inline auto with_lock(std::mutex & mutex, Callable callable) -> decltype(callable())
  {
    std::scoped_lock lock(mutex);
    return callable();
  }

  /**
   * @brief Threadsafe way of modifying the replayer's state using a callback
   *
   * @param modify_cb
   */
  void modify_state_no_lock(const StateModificationCallback & modify_cb);

  /**
   * @brief Modify the replayer's state using a callback, not threadsafe
   *
   * @param modify_cb
   */
  void modify_state(const StateModificationCallback & modify_cb);

  /**
   * @brief Main loop running when replaying
   *
   */
  void play_loop();

  /**
   * @brief Stop the play loop if it is running
   *
   */
  void stop_play_thread();

  /**
   * @brief Get the next index to be played
   *
   * @return std::size_t
   */
  [[nodiscard]] std::size_t get_next_index() const;

  /**
   * @brief Prepare all required data for a given index
   *
   * @param index - Index to be played
   */
  void prepare_data(std::size_t index);

  /**
   * @brief Play all data for a given index
   *
   * @param index - Index to be played
   */
  void play_data(std::size_t index);

  /**
   * @brief Start playing a given index range
   *
   * @param index_range
   * @param replay_speed
   * @return true - Play successfully started
   * @return false - Otherwise
   */
  bool play_index_range(const IndexRange & index_range, float replay_speed);
};

}  // namespace r2k_core

#endif  // ROS2_KITTI_CORE__DATA_REPLAYER_HPP_
