#include <gtest/gtest.h>

#include <memory>
#include <ros2_kitti_replay/clock_data_loader.hpp>
#include <ros2_kitti_replay/load_and_play_data_interface.hpp>
#include <ros2_kitti_replay/timestamp_utils.hpp>
#include <ros2_kitti_replay_test/test_utils.hpp>

namespace
{
using MsgType = r2k_replay::ClockDataLoader::Type;
using ClockInterface = r2k_replay::LoadAndPlayDataInterface<MsgType>;

}  // namespace

TEST(TestLoadAndPlayDataInterface, NormalOperations)
{
  constexpr std::size_t number_timestamps = 3;
  const auto timestamps = r2k_replay_test::generate_test_timestamps(1, number_timestamps);
  ASSERT_EQ(timestamps.size(), number_timestamps);

  auto loader_ptr = std::make_unique<r2k_replay::ClockDataLoader>("clock_data_loader");
  ASSERT_TRUE(loader_ptr->setup(timestamps, ""));
  ASSERT_TRUE(loader_ptr->ready());
  ASSERT_EQ(loader_ptr->data_size(), number_timestamps);

  r2k_replay::Timestamp current_timestamp;

  const ClockInterface::PlayCb play_cb = [&](const auto & clock_msg) {
    current_timestamp = clock_msg.clock;
    return true;
  };

  ClockInterface interface("clock_interface", play_cb, std::move(loader_ptr));
  ASSERT_TRUE(interface.ready());
  ASSERT_EQ(interface.data_size(), number_timestamps);

  for (std::size_t i = 0; i < interface.data_size(); i++) {
    ASSERT_TRUE(interface.prepare(i));
    ASSERT_TRUE(interface.play(i));
    ASSERT_EQ(current_timestamp.nanoseconds(), timestamps.at(i).nanoseconds());
  }

  ASSERT_FALSE(interface.prepare(interface.data_size()));
  ASSERT_FALSE(interface.play(interface.data_size()));
}

TEST(TestLoadAndPlayDataInterface, LoaderNotReady)
{
  auto loader_ptr = std::make_unique<r2k_replay::ClockDataLoader>("clock_data_loader");
  ASSERT_FALSE(loader_ptr->ready());
  ClockInterface interface(
    "clock_interface", []([[maybe_unused]] const auto & clock_msg) { return true; },
    std::move(loader_ptr));
  ASSERT_EQ(interface.data_size(), std::size_t{0});
  ASSERT_FALSE(interface.ready());
  ASSERT_FALSE(interface.prepare(0));
  ASSERT_FALSE(interface.play(0));
}
