#include <gtest/gtest.h>

#include <chrono>
#include <mutex>
#include <ros2_kitti_replay/data_replayer.hpp>
#include <ros2_kitti_replay/play_data_callback_base.hpp>
#include <ros2_kitti_replay/timestamps.hpp>
#include <string>
#include <thread>
#include <vector>

namespace
{
using r2k_replay::DataReplayer;
using r2k_replay::PlayDataCallbackBase;
using r2k_replay::Timestamp;
using r2k_replay::Timestamps;
using PlayRequest = r2k_replay::DataReplayer::PlayRequest;
using ReplayerState = r2k_replay::DataReplayer::ReplayerState;
using StateChangeCallback = r2k_replay::DataReplayer::StateChangeCallback;
using StepRequest = r2k_replay::DataReplayer::StepRequest;

class TestPlayDataCallback final : public PlayDataCallbackBase
{
public:
  using IndexRecord = std::vector<size_t>;

  TestPlayDataCallback(const std::string & name = "Test Play Data Callback")
  : PlayDataCallbackBase(name)
  {
  }

  [[nodiscard]] bool ready() const final { return true; }
  [[nodiscard]] std::size_t data_size() const final { return 0; }
  bool prepare(const std::size_t idx) final
  {
    prepare_record_.push_back(idx);
    return true;
  }
  bool play(const std::size_t idx) final
  {
    play_record_.push_back(idx);
    return true;
  }
  [[nodiscard]] const IndexRecord & prepare_record() const noexcept { return prepare_record_; }
  [[nodiscard]] const IndexRecord & play_record() const noexcept { return play_record_; }

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

class DataReplayerTests : public ::testing::Test
{
public:
  static constexpr size_t kNumberTimestamps = 10;
  static constexpr size_t kStartTimeSeconds = 2;
  static constexpr auto kTimestampIntervalNs = static_cast<size_t>(1e7);
  static const Timestamps kTimestamps;

  StateChangeCallback get_state_change_callback()
  {
    return [this](const ReplayerState & replayer_state) {
      std::scoped_lock lock(state_mutex_);
      replayer_states_.push_back(replayer_state);
    };
  }

  void assert_timeline_played_exactly_once(
    const size_t start_index = 0, const Timestamps & timestamps = kTimestamps) const
  {
    const auto & play_data_callback = *play_cb_ptr;
    const auto num_stamps = timestamps.size();
    ASSERT_EQ(play_data_callback.prepare_record().size(), num_stamps);
    ASSERT_EQ(play_data_callback.play_record().size(), num_stamps);
    for (size_t i = 0; i < num_stamps; i++) {
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
    const size_t start_index = 0, const Timestamps & timestamps = kTimestamps) const
  {
    const auto & play_data_callback = *play_cb_ptr;
    const auto num_stamps = timestamps.size();
    const auto prepare_size = play_cb_ptr->prepare_record().size();
    const auto play_size = play_cb_ptr->play_record().size();
    EXPECT_TRUE((prepare_size > 0) && (prepare_size < num_stamps));
    EXPECT_TRUE((play_size > 0) && (play_size < num_stamps));
    ASSERT_EQ(play_data_callback.play_record().at(0), start_index);
    ASSERT_EQ(play_data_callback.prepare_record().at(0), start_index);
    const auto last_state = get_last_replayer_state();
    ASSERT_EQ(last_state.playing, false);
    ASSERT_LT(last_state.next_idx, num_stamps);
  }

  void wait_until(
    const std::function<bool(void)> & condition,
    const size_t check_interval_ns = kTimestampIntervalNs)
  {
    while (condition()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(check_interval_ns));
    }
  }

  void wait_till_replayer_no_longer_playing(const size_t check_interval_ns = kTimestampIntervalNs)
  {
    wait_until([this]() { return replayer.is_playing(); }, check_interval_ns);
  }

  [[nodiscard]] std::vector<ReplayerState> get_replayer_states() const
  {
    std::scoped_lock lock(state_mutex_);
    return replayer_states_;
  }

  [[nodiscard]] ReplayerState get_last_replayer_state() const
  {
    std::scoped_lock lock(state_mutex_);
    return replayer_states_.back();
  }

  DataReplayer replayer{"Test Replayer", kTimestamps};
  std::shared_ptr<TestPlayDataCallback> play_cb_ptr = std::make_shared<TestPlayDataCallback>();

  void play_timeline_halfway(const std::function<void()> & interrupt_play_cb)
  {
    ASSERT_TRUE(replayer.play(PlayRequest(kTimestamps.front(), kTimestamps.back())));
    ASSERT_TRUE(replayer.is_playing());
    std::this_thread::sleep_for(
      std::chrono::nanoseconds(kNumberTimestamps / size_t{2} * kTimestampIntervalNs));
    ASSERT_TRUE(replayer.is_playing());
    interrupt_play_cb();
    ASSERT_FALSE(replayer.is_playing());
    assert_timeline_played_partially();
  }

  void assert_replayer_has_been_reset_after_cb(const std::function<void()> & reset_cb)
  {
    const auto paused_state = get_last_replayer_state();
    ASSERT_GT(paused_state.current_time, kTimestamps.front());
    ASSERT_GT(paused_state.next_idx, size_t{0});

    reset_cb();

    const auto stopped_state = get_last_replayer_state();
    ASSERT_EQ(stopped_state.current_time, kTimestamps.front());
    ASSERT_EQ(stopped_state.next_idx, size_t{0});
  }

protected:
  mutable std::mutex state_mutex_;
  std::vector<ReplayerState> replayer_states_;

  void SetUp()
  {
    ASSERT_TRUE(replayer.add_play_data_cb(play_cb_ptr));
    ASSERT_TRUE(replayer.set_state_change_cb(get_state_change_callback()));
  }

  [[nodiscard]] static Timestamps generate_test_timestamps()
  {
    Timestamps output;
    output.reserve(kNumberTimestamps);
    for (size_t i = 0; i < kNumberTimestamps; i++) {
      output.emplace_back(kStartTimeSeconds, i * kTimestampIntervalNs);
    }
    return output;
  }
};
const Timestamps DataReplayerTests::kTimestamps = DataReplayerTests::generate_test_timestamps();

TEST_F(DataReplayerTests, NormalInitializationTest)
{
  const auto state = replayer.get_replayer_state();
  ASSERT_EQ(state.start_time, kTimestamps.front());
  ASSERT_EQ(state.final_time, kTimestamps.back());
  ASSERT_EQ(state.data_size, kTimestamps.size());
}

TEST_F(DataReplayerTests, EmptyTimestampInitializationTest)
{
  auto empty_replayer = DataReplayer("Test Replayer", Timestamps{});
  ASSERT_EQ(empty_replayer.get_replayer_state(), ReplayerState());
}

TEST_F(DataReplayerTests, PlayTest)
{
  ASSERT_TRUE(replayer.play(PlayRequest(kTimestamps.front(), kTimestamps.back())));
  wait_till_replayer_no_longer_playing();
  assert_timeline_played_exactly_once();
}

TEST_F(DataReplayerTests, ResumeTest)
{
  ASSERT_TRUE(replayer.resume());
  wait_till_replayer_no_longer_playing();
  ASSERT_FALSE(replayer.resume());
  assert_timeline_played_exactly_once();
}

TEST_F(DataReplayerTests, DestructorTest)
{
  {
    auto scoped_replayer = DataReplayer{"Scoped Test Replayer", kTimestamps};
    ASSERT_TRUE(scoped_replayer.add_play_data_cb(play_cb_ptr));
    ASSERT_TRUE(scoped_replayer.set_state_change_cb(get_state_change_callback()));
    ASSERT_TRUE(scoped_replayer.resume());
    std::this_thread::sleep_for(std::chrono::nanoseconds(kTimestampIntervalNs));
  }
  assert_timeline_played_partially();
}

TEST_F(DataReplayerTests, PlaySegmentTest)
{
  constexpr size_t start_index = 3;
  constexpr size_t segment_length = 3;
  ASSERT_LT(start_index + segment_length, kTimestamps.size());
  const Timestamps truncated_timestamp{
    &kTimestamps[start_index], &kTimestamps[start_index + segment_length]};

  ASSERT_TRUE(replayer.play(PlayRequest(truncated_timestamp.front(), truncated_timestamp.back())));
  wait_till_replayer_no_longer_playing();
  assert_timeline_played_exactly_once(start_index, truncated_timestamp);
}

TEST_F(DataReplayerTests, PauseWhileNotPlayingTest)
{
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.pause());
  ASSERT_FALSE(replayer.is_playing());
}

TEST_F(DataReplayerTests, PauseWhilePlayingTest)
{
  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.pause()); });

  const auto paused_state = get_last_replayer_state();
  const auto resume_idx = paused_state.next_idx;
  play_cb_ptr->reset();
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.resume());

  wait_till_replayer_no_longer_playing();
  ASSERT_FALSE(replayer.is_playing());
  assert_timeline_played_exactly_once(
    resume_idx, Timestamps{std::next(kTimestamps.cbegin(), resume_idx), kTimestamps.cend()});
}

TEST_F(DataReplayerTests, StopWhilePlayingTest)
{
  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.stop()); });

  const auto stopped_state = get_last_replayer_state();
  ASSERT_EQ(stopped_state.current_time, kTimestamps.front());
  ASSERT_EQ(stopped_state.next_idx, size_t{0});
}

TEST_F(DataReplayerTests, StopWhileNotPlayingTest)
{
  ASSERT_FALSE(replayer.is_playing());
  ASSERT_TRUE(replayer.stop());
  ASSERT_FALSE(replayer.is_playing());

  play_timeline_halfway([&]() { ASSERT_TRUE(replayer.pause()); });
  assert_replayer_has_been_reset_after_cb([&]() { ASSERT_TRUE(replayer.stop()); });
}

TEST_F(DataReplayerTests, ResetTest)
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

class DataReplayerSpeedFactorTests : public DataReplayerTests,
                                     public ::testing::WithParamInterface<float>
{
public:
  static constexpr auto kExpectedNormalPlayDurationNs = kNumberTimestamps * kTimestampIntervalNs;
  static constexpr float kTol = 0.2;
};

TEST_P(DataReplayerSpeedFactorTests, SpeedFactorTests)
{
  const auto speed_factor = GetParam();

  auto start_time = std::chrono::high_resolution_clock::now();
  ASSERT_TRUE(replayer.resume(speed_factor));
  wait_till_replayer_no_longer_playing();
  auto end_time = std::chrono::high_resolution_clock::now();
  const auto time_taken =
    std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
  using DurationType = std::decay_t<decltype(time_taken)>;

  const bool play_as_fast_as_possible = speed_factor <= 0.0;
  if (play_as_fast_as_possible) {
    ASSERT_LT(time_taken, static_cast<DurationType>(kExpectedNormalPlayDurationNs));
  } else {
    const auto expected_duration_ns =
      static_cast<DurationType>(kExpectedNormalPlayDurationNs * (1.0 / speed_factor));
    EXPECT_TRUE(
      (time_taken > static_cast<DurationType>(expected_duration_ns * (1.0 - kTol))) &&
      (time_taken < static_cast<DurationType>(expected_duration_ns * (1.0 + kTol))));
  }
}

INSTANTIATE_TEST_SUITE_P(
  DataReplayerSpeedFactorTimingTests, DataReplayerSpeedFactorTests,
  ::testing::Values(-100.0, -1.0, -0.001, 0.0, 0.1, 0.5, 1.0, 1.1, 2.0, 5.0));
