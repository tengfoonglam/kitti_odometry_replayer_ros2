#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <iterator>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_core/data_replayer.hpp>
#include <ros2_kitti_core/play_data_interface_base.hpp>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <ros2_kitti_core_test/test_utils.hpp>
#include <string>
#include <thread>
#include <vector>

namespace
{
using r2k_core::DataReplayer;
using r2k_core::PlayDataInterfaceBase;
using r2k_core::Timestamp;
using r2k_core::Timestamps;
using TimeRange = r2k_core::DataReplayer::TimeRange;
using ReplayerState = r2k_core::DataReplayer::ReplayerState;
using StateChangeCallback = r2k_core::DataReplayer::StateChangeCallback;
using StepRequest = r2k_core::DataReplayer::StepRequest;

class PlayDataTestInterface final : public PlayDataInterfaceBase
{
public:
  using IndexRecord = std::vector<std::size_t>;

  PlayDataTestInterface(const std::string & name = "Test Play Data Callback")
  : PlayDataInterfaceBase(name)
  {
  }

  bool ready() const final { return true; }
  std::size_t data_size() const final { return 0; }
  bool prepare(std::size_t idx) final
  {
    prepare_record_.push_back(idx);
    return true;
  }
  bool play(std::size_t idx) final
  {
    play_record_.push_back(idx);
    return true;
  }
  const IndexRecord & prepare_record() const noexcept { return prepare_record_; }
  const IndexRecord & play_record() const noexcept { return play_record_; }

  void reset()
  {
    play_record_.clear();
    prepare_record_.clear();
  }

private:
  IndexRecord play_record_;
  IndexRecord prepare_record_;
};

}  // namespace

class TestDataReplayer : public ::testing::Test
{
public:
  static constexpr std::size_t kNumberTimestamps{10};
  static constexpr std::size_t kStartTimeSeconds{2};
  static constexpr auto kTimestampIntervalNs{static_cast<size_t>(1e7)};
  static constexpr auto kCheckIntervalNs{kTimestampIntervalNs / 100};
  static const Timestamps kTimestamps;

  StateChangeCallback get_state_change_callback()
  {
    return [this](const ReplayerState & replayer_state) {
      std::scoped_lock lock(state_mutex_);
      replayer_states_.push_back(replayer_state);
    };
  }

  void assert_timeline_played_exactly_once(
    std::size_t start_index = 0, const Timestamps & timestamps = kTimestamps) const
  {
    const auto & play_data_callback = *play_interface_ptr;
    const auto num_stamps = timestamps.size();
    ASSERT_EQ(play_data_callback.prepare_record().size(), num_stamps);
    ASSERT_EQ(play_data_callback.play_record().size(), num_stamps);
    for (std::size_t i = 0; i < num_stamps; i++) {
      ASSERT_EQ(play_data_callback.play_record().at(i), start_index + i);
      ASSERT_EQ(play_data_callback.prepare_record().at(i), start_index + i);
    }
    const auto last_state = get_last_replayer_state();
    ASSERT_EQ(last_state.playing, false);
    ASSERT_GE(last_state.current_time, timestamps.back());
    ASSERT_EQ(last_state.next_idx, start_index + num_stamps);
    ASSERT_EQ(last_state.target_idx, start_index + num_stamps - 1);
  }

  void assert_timeline_played_partially(
    std::size_t start_index = 0, const Timestamps & timestamps = kTimestamps) const
  {
    const auto & play_data_callback = *play_interface_ptr;
    const auto num_stamps = timestamps.size();
    const auto prepare_size = play_interface_ptr->prepare_record().size();
    const auto play_size = play_interface_ptr->play_record().size();
    EXPECT_TRUE((prepare_size > 0) && (prepare_size < num_stamps));
    EXPECT_TRUE((play_size > 0) && (play_size < num_stamps));
    ASSERT_EQ(play_data_callback.play_record().at(0), start_index);
    ASSERT_EQ(play_data_callback.prepare_record().at(0), start_index);
    const auto last_state = get_last_replayer_state();
    ASSERT_EQ(last_state.playing, false);
    ASSERT_LT(last_state.next_idx, num_stamps);
  }

  static void wait_until(
    const std::function<bool(void)> & condition, std::size_t check_interval_ns = kCheckIntervalNs)
  {
    while (condition()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(check_interval_ns));
    }
  }

  static void wait_till_replayer_no_longer_playing(const DataReplayer & replayer)
  {
    wait_until(
      [&replayer = std::as_const(replayer)]() { return replayer.is_playing(); }, kCheckIntervalNs);
  }

  std::vector<ReplayerState> get_replayer_states() const
  {
    std::scoped_lock lock(state_mutex_);
    return replayer_states_;
  }

  ReplayerState get_last_replayer_state() const
  {
    std::scoped_lock lock(state_mutex_);
    return replayer_states_.back();
  }

  DataReplayer replayer{"Test Replayer", kTimestamps};
  std::shared_ptr<PlayDataTestInterface> play_interface_ptr =
    std::make_shared<PlayDataTestInterface>();

  void play_timeline_halfway(const std::function<void()> & interrupt_play_cb)
  {
    ASSERT_TRUE(
      replayer.set_next_play_time_range(TimeRange(kTimestamps.front(), kTimestamps.back())));
    ASSERT_TRUE(replayer.play());
    ASSERT_TRUE(replayer.is_playing());
    std::this_thread::sleep_for(
      std::chrono::nanoseconds(kNumberTimestamps / std::size_t{2} * kTimestampIntervalNs));
    ASSERT_TRUE(replayer.is_playing());
    interrupt_play_cb();
    ASSERT_FALSE(replayer.is_playing());
    assert_timeline_played_partially();
  }

  void assert_replayer_has_been_reset_after_cb(const std::function<void()> & reset_cb)
  {
    const auto paused_state = get_last_replayer_state();
    ASSERT_GT(paused_state.current_time, kTimestamps.front());
    ASSERT_GT(paused_state.next_idx, std::size_t{0});

    reset_cb();

    const auto stopped_state = get_last_replayer_state();
    ASSERT_EQ(stopped_state.current_time, kTimestamps.front());
    ASSERT_EQ(stopped_state.next_idx, std::size_t{0});
  }

protected:
  mutable std::mutex state_mutex_;
  std::vector<ReplayerState> replayer_states_;

  void SetUp()
  {
    ASSERT_TRUE(replayer.add_play_data_interface(play_interface_ptr));
    ASSERT_TRUE(replayer.set_state_change_cb(get_state_change_callback()));
  }

  static Timestamps generate_test_timestamps()
  {
    Timestamps output;
    output.reserve(kNumberTimestamps);
    for (size_t i = 0; i < kNumberTimestamps; i++) {
      output.emplace_back(kStartTimeSeconds, i * kTimestampIntervalNs);
    }
    return output;
  }
};
const Timestamps TestDataReplayer::kTimestamps = TestDataReplayer::generate_test_timestamps();

TEST_F(TestDataReplayer, NormalInitializationTest)
{
  const auto state = replayer.get_replayer_state();
  ASSERT_EQ(state.start_time, kTimestamps.front());
  ASSERT_EQ(state.end_time, kTimestamps.back());
  ASSERT_EQ(state.end_idx, kTimestamps.size());
}

TEST_F(TestDataReplayer, EmptyTimestampInitializationTest)
{
  auto empty_replayer = DataReplayer("Test Replayer", Timestamps{});
  ASSERT_EQ(empty_replayer.get_replayer_state(), ReplayerState());
}

TEST_F(TestDataReplayer, PlayTest)
{
  ASSERT_TRUE(replayer.play());
  wait_till_replayer_no_longer_playing(replayer);
  ASSERT_FALSE(replayer.play());
  assert_timeline_played_exactly_once();
}

TEST_F(TestDataReplayer, SetTimeRangeBasicTest)
{
  ASSERT_TRUE(
    replayer.set_next_play_time_range(TimeRange(kTimestamps.front(), kTimestamps.back())));
  ASSERT_TRUE(replayer.play());
  wait_till_replayer_no_longer_playing(replayer);
  assert_timeline_played_exactly_once();
}

TEST_F(TestDataReplayer, DestructorTest)
{
  {
    auto scoped_replayer = DataReplayer{"Scoped Test Replayer", kTimestamps};
    ASSERT_TRUE(scoped_replayer.add_play_data_interface(play_interface_ptr));
    ASSERT_TRUE(scoped_replayer.set_state_change_cb(get_state_change_callback()));
    ASSERT_TRUE(scoped_replayer.play());
    std::this_thread::sleep_for(std::chrono::nanoseconds(kTimestampIntervalNs));
  }
  assert_timeline_played_partially();
}

TEST_F(TestDataReplayer, SetTimeRangeSegmentTest)
{
  constexpr std::size_t start_index{3};
  constexpr std::size_t segment_length{3};
  ASSERT_LT(start_index + segment_length, kTimestamps.size());
  const Timestamps truncated_timestamp{
    &kTimestamps[start_index], &kTimestamps[start_index + segment_length]};

  ASSERT_TRUE(replayer.set_next_play_time_range(
    TimeRange(truncated_timestamp.front(), truncated_timestamp.back())));
  ASSERT_TRUE(replayer.play());
  wait_till_replayer_no_longer_playing(replayer);
  assert_timeline_played_exactly_once(start_index, truncated_timestamp);
}

TEST_F(TestDataReplayer, PauseWhileNotPlayingTest)
{
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.pause());
  ASSERT_FALSE(replayer.is_playing());
}

TEST_F(TestDataReplayer, PauseWhilePlayingTest)
{
  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.pause()); });

  const auto paused_state = get_last_replayer_state();
  const auto resume_idx = paused_state.next_idx;
  play_interface_ptr->reset();
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.play());

  wait_till_replayer_no_longer_playing(replayer);
  ASSERT_FALSE(replayer.is_playing());
  assert_timeline_played_exactly_once(
    resume_idx,
    Timestamps{std::next(std::cbegin(kTimestamps), resume_idx), std::cend(kTimestamps)});
}

TEST_F(TestDataReplayer, StopWhilePlayingTest)
{
  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.stop()); });

  const auto stopped_state = get_last_replayer_state();
  ASSERT_EQ(stopped_state.current_time, kTimestamps.front());
  ASSERT_EQ(stopped_state.next_idx, std::size_t{0});
}

TEST_F(TestDataReplayer, StopWhileNotPlayingTest)
{
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.stop());
  ASSERT_FALSE(replayer.is_playing());

  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.pause()); });
  assert_replayer_has_been_reset_after_cb([&]() { ASSERT_TRUE(replayer.stop()); });
}

TEST_F(TestDataReplayer, ResetTest)
{
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.reset());
  ASSERT_FALSE(replayer.is_playing());

  play_timeline_halfway([&]() {
    ASSERT_FALSE(replayer.reset());
    ASSERT_TRUE(replayer.pause());
  });

  assert_replayer_has_been_reset_after_cb([&]() { ASSERT_TRUE(replayer.reset()); });
}

TEST_F(TestDataReplayer, StepThenPlayTest)
{
  constexpr std::size_t numberSteps{3};
  ASSERT_LT(numberSteps, kNumberTimestamps);
  ASSERT_TRUE(replayer.step(StepRequest{numberSteps}));
  wait_till_replayer_no_longer_playing(replayer);
  ASSERT_TRUE(replayer.play());
  wait_till_replayer_no_longer_playing(replayer);
  assert_timeline_played_exactly_once();
}

TEST_F(TestDataReplayer, TruncatedTimestampInitialization)
{
  static constexpr std::size_t kStartIdx{2};
  static constexpr std::size_t kTargetIdx{4};
  const Timestamp kStartTime{kTimestamps.at(kStartIdx)};
  const Timestamp kTargetTime{kTimestamps.at(kTargetIdx)};
  const Timestamp kEndTime{kTargetTime};
  const Timestamps truncated_timestamps(
    std::cbegin(kTimestamps) + kStartIdx, std::cbegin(kTimestamps) + kTargetIdx + 1);

  const auto assert_expected_initial_truncated_state = [&](const auto & state) {
    ASSERT_EQ(state.start_idx, kStartIdx);
    ASSERT_EQ(state.next_idx, kStartIdx);
    ASSERT_EQ(state.start_time, kStartTime);
    ASSERT_EQ(state.target_idx, kTargetIdx);
    ASSERT_EQ(state.target_time, kTargetTime);
    ASSERT_EQ(state.current_time, kStartTime);
    ASSERT_EQ(state.end_time, kEndTime);
  };

  DataReplayer truncated_replayer{
    "Test Replayer", kTimestamps, rclcpp::get_logger("Test Replayer"),
    TimeRange(kStartTime, kTargetTime)};
  ASSERT_TRUE(truncated_replayer.add_play_data_interface(play_interface_ptr));
  ASSERT_TRUE(truncated_replayer.set_state_change_cb(get_state_change_callback()));
  assert_expected_initial_truncated_state(truncated_replayer.get_replayer_state());
  ASSERT_TRUE(truncated_replayer.play());
  wait_till_replayer_no_longer_playing(truncated_replayer);
  assert_timeline_played_exactly_once(kStartIdx, truncated_timestamps);
  truncated_replayer.reset();
  assert_expected_initial_truncated_state(truncated_replayer.get_replayer_state());
}

class TestDataReplayerSpeedFactor : public TestDataReplayer,
                                    public ::testing::WithParamInterface<std::tuple<float, bool>>
{
public:
  static constexpr auto kExpectedNormalPlayDurationNs{kNumberTimestamps * kTimestampIntervalNs};
  static constexpr std::size_t kNumberSteps{8};
  static constexpr auto kExpectedNormalStepDurationNs{kNumberSteps * kTimestampIntervalNs};
  static constexpr float kTol{0.2};

  static_assert(
    kNumberSteps < kNumberTimestamps, "Step size must be smaller the total number of messages");
};

TEST_P(TestDataReplayerSpeedFactor, SpeedFactorTests)
{
  const auto [speed_factor, run_play] = GetParam();
  const auto expected_normal_duration =
    run_play ? kExpectedNormalPlayDurationNs : kExpectedNormalStepDurationNs;

  const auto start_time = std::chrono::high_resolution_clock::now();

  if (run_play) {
    ASSERT_TRUE(replayer.play(speed_factor));
  } else {
    ASSERT_TRUE(replayer.step(StepRequest{kNumberSteps, speed_factor}));
  }

  wait_till_replayer_no_longer_playing(replayer);
  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto time_taken =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
  using DurationType = std::decay_t<decltype(time_taken)>;

  const bool play_as_fast_as_possible = speed_factor <= 0.0;
  if (play_as_fast_as_possible) {
    ASSERT_LT(time_taken, static_cast<DurationType>(expected_normal_duration));
  } else {
    const auto expected_duration_ns =
      static_cast<DurationType>(expected_normal_duration * (1.0 / speed_factor));
    EXPECT_TRUE(
      (time_taken > static_cast<DurationType>(expected_duration_ns * (1.0 - kTol))) &&
      (time_taken < static_cast<DurationType>(expected_duration_ns * (1.0 + kTol))));
  }
}

INSTANTIATE_TEST_SUITE_P(
  DataReplayerSpeedFactorTimingTests, TestDataReplayerSpeedFactor,
  ::testing::Combine(
    ::testing::ValuesIn(std::vector<float>{
      -100.0, -1.0, -0.001, 0.0, 0.1, 0.5, 1.0, 1.1, 2.0, 5.0}),
    ::testing::ValuesIn(std::vector<bool>{false, true})));
