#include <gtest/gtest.h>

#include <ros2_kitti_replay/data_replayer.hpp>
#include <ros2_kitti_replay/timestamps.hpp>
#include <tuple>

namespace
{
using r2k_replay::DataReplayer;
using r2k_replay::Timestamp;
using r2k_replay::Timestamps;
using PlayRequest = DataReplayer::PlayRequest;
using IndexRange = DataReplayer::IndexRange;
}  // namespace

class DataReplayerStaticTests : public ::testing::Test
{
public:
  static void assert_optional_index_range_equal(
    const DataReplayer::IndexRangeOpt & lhs, const DataReplayer::IndexRangeOpt & rhs)
  {
    ASSERT_EQ(lhs.has_value(), rhs.has_value());
    if (lhs.has_value() && rhs.has_value()) {
      ASSERT_EQ(lhs.value(), rhs.value());
    }
  }
};

class ProcessPlayRequestNormalOperationsTests
: public DataReplayerStaticTests,
  public ::testing::WithParamInterface<
    std::tuple<PlayRequest, Timestamps, std::optional<DataReplayer::IndexRange>>>
{
public:
  static const Timestamps kTimestamps;

private:
  [[nodiscard]] static Timestamps generate_test_timestamps()
  {
    Timestamps output;
    for (size_t i = 1; i <= 5; i++) {
      output.emplace_back(i, 0);
    }
    return output;
  }
};
const Timestamps ProcessPlayRequestNormalOperationsTests::kTimestamps =
  ProcessPlayRequestNormalOperationsTests::generate_test_timestamps();

namespace
{
const auto & kTimestamps = ProcessPlayRequestNormalOperationsTests::kTimestamps;
}  // namespace

TEST_P(ProcessPlayRequestNormalOperationsTests, NormalOperationsTests)
{
  const auto [request, timestamps, answer] = GetParam();
  const auto output = DataReplayer::process_play_request(request, timestamps);
  assert_optional_index_range_equal(answer, output);
}

INSTANTIATE_TEST_SUITE_P(
  TestProcessPlayRequest, ProcessPlayRequestNormalOperationsTests,
  ::testing::Values(
    std::make_tuple(
      PlayRequest(kTimestamps.front(), kTimestamps.back()), kTimestamps,
      std::optional(std::make_tuple(0, kTimestamps.size() - 1))),
    std::make_tuple(
      PlayRequest(kTimestamps.front(), kTimestamps.front()), kTimestamps,
      std::optional(std::make_tuple(0, 0))),
    std::make_tuple(PlayRequest(Timestamp(0, 5), Timestamp(0, 5)), kTimestamps, std::nullopt),
    std::make_tuple(
      PlayRequest(Timestamp(0, 5), Timestamp(1, 5)), kTimestamps,
      std::optional(std::make_tuple(0, 0))),
    std::make_tuple(
      PlayRequest(Timestamp(0, 0), Timestamp(6, 0)), kTimestamps,
      std::optional(std::make_tuple(0, kTimestamps.size() - 1))),
    std::make_tuple(
      PlayRequest(Timestamp(2, 5), Timestamp(4, 5)), kTimestamps,
      std::optional(std::make_tuple(2, 3))),
    std::make_tuple(
      PlayRequest(Timestamp(2, 5), Timestamp(6, 0)), kTimestamps,
      std::optional(std::make_tuple(2, kTimestamps.size() - 1))),
    std::make_tuple(
      PlayRequest(Timestamp(0, 0), Timestamp(3, 5)), kTimestamps,
      std::optional(std::make_tuple(0, 2))),
    std::make_tuple(
      PlayRequest(Timestamp(2, 0), Timestamp(2, 0)), kTimestamps,
      std::optional(std::make_tuple(1, 1))),
    std::make_tuple(
      PlayRequest(kTimestamps.back(), kTimestamps.back()), kTimestamps,
      std::optional(std::make_tuple(kTimestamps.size() - 1, kTimestamps.size() - 1)))));

TEST(TestProcessPlayRequest, EmptyTimestampTests)
{
  const auto index_opt =
    DataReplayer::process_play_request(PlayRequest(Timestamp(), Timestamp(1)), Timestamps{});
  ASSERT_FALSE(index_opt.has_value());
}
