#include <gtest/gtest.h>

#include <memory>
#include <ros2_kitti_core/clock_data_loader.hpp>
#include <ros2_kitti_core/load_and_play_data_interface.hpp>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <ros2_kitti_core_test/test_utils.hpp>

namespace
{
using r2k_core::ClockDataLoader;
using r2k_core::LoadAndPlayDataInterface;

}  // namespace

TEST(TestLoadAndPlayDataInterface, NormalOperations)
{
  constexpr std::size_t number_timestamps = 3;
  const auto timestamps = r2k_core_test::generate_test_timestamps(1, number_timestamps);
  ASSERT_EQ(timestamps.size(), number_timestamps);

  auto loader_ptr = std::make_unique<ClockDataLoader>("clock_data_loader");
  ASSERT_TRUE(loader_ptr->setup(timestamps, ""));
  ASSERT_TRUE(loader_ptr->ready());
  ASSERT_EQ(loader_ptr->data_size(), number_timestamps);

  r2k_core::Timestamp current_timestamp;

  const auto play_cb = [&](const auto & clock_msg) {
    current_timestamp = clock_msg.clock;
    return true;
  };

  LoadAndPlayDataInterface interface("clock_interface", play_cb, std::move(loader_ptr));
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
  auto loader_ptr = std::make_unique<ClockDataLoader>("clock_data_loader");
  ASSERT_FALSE(loader_ptr->ready());
  LoadAndPlayDataInterface interface(
    "clock_interface", []([[maybe_unused]] const auto & clock_msg) { return true; },
    std::move(loader_ptr));
  ASSERT_EQ(interface.data_size(), std::size_t{0});
  ASSERT_FALSE(interface.ready());
  ASSERT_FALSE(interface.prepare(0));
  ASSERT_FALSE(interface.play(0));
}
