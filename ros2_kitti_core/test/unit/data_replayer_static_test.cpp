#include <gtest/gtest.h>

#include <ros2_kitti_core/data_replayer.hpp>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <ros2_kitti_core_test/test_utils.hpp>
#include <tuple>

namespace
{
using r2k_core::DataReplayer;
using r2k_core::Timestamp;
using r2k_core::Timestamps;
using TimeRange = DataReplayer::TimeRange;
using StepRequest = DataReplayer::StepRequest;
using IndexRangeOpt = DataReplayer::IndexRangeOpt;
}  // namespace

class TestDataReplayerStatic : public ::testing::Test
{
public:
  static void assert_optional_index_range_equal(
    const IndexRangeOpt & lhs, const IndexRangeOpt & rhs)
  {
    ASSERT_EQ(lhs.has_value(), rhs.has_value());
    if (lhs.has_value() && rhs.has_value()) {
      ASSERT_EQ(lhs.value(), rhs.value());
    }
  }
};

class TestProcessTimeRangeNormalOperations
: public TestDataReplayerStatic,
  public ::testing::WithParamInterface<std::tuple<TimeRange, Timestamps, IndexRangeOpt>>
{
public:
  static const Timestamps kTimestamps;
  static constexpr std::size_t kStartTimeS{1};
  static constexpr std::size_t kEndTimeS{5};
};
const Timestamps TestProcessTimeRangeNormalOperations::kTimestamps =
  r2k_core_test::generate_test_timestamps(
    TestProcessTimeRangeNormalOperations::kStartTimeS,
    TestProcessTimeRangeNormalOperations::kEndTimeS);

namespace
{
const auto & kTimestamps = TestProcessTimeRangeNormalOperations::kTimestamps;
}  // namespace

TEST_P(TestProcessTimeRangeNormalOperations, NormalOperationsTests)
{
  const auto [request, timestamps, answer] = GetParam();
  static constexpr std::size_t start_idx{0};
  ASSERT_GT(timestamps.size(), std::size_t{0});
  const std::size_t end_idx{timestamps.size()};
  const auto output =
    DataReplayer::process_set_next_play_time_range_request(request, start_idx, end_idx, timestamps);
  assert_optional_index_range_equal(answer, output);
}

INSTANTIATE_TEST_SUITE_P(
  TestDataReplayerStatic, TestProcessTimeRangeNormalOperations,
  ::testing::Values(
    std::make_tuple(
      TimeRange(kTimestamps.front(), kTimestamps.back()), kTimestamps,
      std::optional(std::make_tuple(0, kTimestamps.size() - 1))),
    std::make_tuple(
      TimeRange(kTimestamps.front(), kTimestamps.front()), kTimestamps,
      std::optional(std::make_tuple(0, 0))),
    std::make_tuple(TimeRange(Timestamp(0, 5), Timestamp(0, 5)), kTimestamps, std::nullopt),
    std::make_tuple(
      TimeRange(Timestamp(0, 5), Timestamp(1, 5)), kTimestamps,
      std::optional(std::make_tuple(0, 0))),
    std::make_tuple(
      TimeRange(Timestamp(0, 0), Timestamp(6, 0)), kTimestamps,
      std::optional(std::make_tuple(0, kTimestamps.size() - 1))),
    std::make_tuple(
      TimeRange(Timestamp(2, 5), Timestamp(4, 5)), kTimestamps,
      std::optional(std::make_tuple(2, 3))),
    std::make_tuple(
      TimeRange(Timestamp(2, 5), Timestamp(6, 0)), kTimestamps,
      std::optional(std::make_tuple(2, kTimestamps.size() - 1))),
    std::make_tuple(
      TimeRange(Timestamp(0, 0), Timestamp(3, 5)), kTimestamps,
      std::optional(std::make_tuple(0, 2))),
    std::make_tuple(
      TimeRange(Timestamp(2, 0), Timestamp(2, 0)), kTimestamps,
      std::optional(std::make_tuple(1, 1))),
    std::make_tuple(
      TimeRange(kTimestamps.back(), kTimestamps.back()), kTimestamps,
      std::optional(std::make_tuple(kTimestamps.size() - 1, kTimestamps.size() - 1)))));

TEST(TestDataReplayerStatic, EmptyTimestampTests)
{
  static const Timestamps empty_timestamps{};
  static constexpr std::size_t start_idx{0};
  static constexpr std::size_t end_idx{0};

  const auto index_opt = DataReplayer::process_set_next_play_time_range_request(
    TimeRange(Timestamp(), Timestamp(1)), start_idx, end_idx, empty_timestamps);
  ASSERT_FALSE(index_opt.has_value());
}

// TODO(tf) Add test cases for invalid start, end index values
// TODO(tf) Add test cases for cropping due to non-zero start, end index values

class TestProcessStepRequestNormalOperations
: public TestDataReplayerStatic,
  public ::testing::WithParamInterface<std::tuple<StepRequest, size_t, size_t, IndexRangeOpt>>
{
};

TEST_P(TestProcessStepRequestNormalOperations, NormalOperationsTests)
{
  const auto [request, next_idx, end_idx, answer] = GetParam();
  const auto output = DataReplayer::process_step_request(request, next_idx, end_idx);
  assert_optional_index_range_equal(answer, output);
}

INSTANTIATE_TEST_SUITE_P(
  TestDataReplayerStatic, TestProcessStepRequestNormalOperations,
  ::testing::Values(
    std::make_tuple(StepRequest(0), 1, 3, std::nullopt),
    std::make_tuple(StepRequest(1), 3, 3, std::nullopt),
    std::make_tuple(StepRequest(1), 100, 3, std::nullopt),
    std::make_tuple(StepRequest(1), 1, 3, std::optional(std::make_tuple(1, 1))),
    std::make_tuple(StepRequest(5), 3, 10, std::optional(std::make_tuple(3, 7))),
    std::make_tuple(StepRequest(1), 2, 3, std::optional(std::make_tuple(2, 2))),
    std::make_tuple(StepRequest(100), 2, 10, std::optional(std::make_tuple(2, 9)))));
