#include <gtest/gtest.h>

#include <ros2_kitti_replay/data_replayer.hpp>
#include <ros2_kitti_replay/play_data_callback_base.hpp>
#include <ros2_kitti_replay/timestamps.hpp>
#include <string>
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

private:
  IndexRecord play_record_;
  IndexRecord prepare_record_;
};

}  // namespace

class DataReplayerTests : public ::testing::Test
{
public:
  static constexpr size_t kNumberTimestamps = 5;
  static constexpr size_t kStartTimeSeconds = 1;
  static const Timestamps kTimestamps;

  StateChangeCallback get_state_change_callback()
  {
    return [&replay_states_ = replay_states_](const ReplayerState & replayer_state) {
      replay_states_.push_back(replayer_state);
    };
  }

private:
  std::vector<ReplayerState> replay_states_;

  [[nodiscard]] static Timestamps generate_test_timestamps()
  {
    Timestamps output;
    for (size_t i = 0; i < kNumberTimestamps; i++) {
      output.emplace_back(kStartTimeSeconds + i, 0);
    }
    return output;
  }
};
const Timestamps DataReplayerTests::kTimestamps = DataReplayerTests::generate_test_timestamps();

namespace
{
const auto & kTimestamps = DataReplayerTests::kTimestamps;
}  // namespace

TEST(DataReplayerTests, NormalInitializationTest)
{
  auto replayer = DataReplayer("Test Replayer", kTimestamps);
  const auto state = replayer.get_replayer_state();
  ASSERT_EQ(state.start_time, kTimestamps.front());
  ASSERT_EQ(state.final_time, kTimestamps.back());
  ASSERT_EQ(state.data_size, kTimestamps.size());
}
