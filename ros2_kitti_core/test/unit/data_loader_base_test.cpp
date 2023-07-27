#include <gtest/gtest.h>

#include <cstdint>
#include <ros2_kitti_core/data_loader_base.hpp>
#include <ros2_kitti_core/timestamp_utils.hpp>
#include <string>
#include <utility>

class TestDataLoaderBase : public ::testing::Test
{
public:
  static constexpr const char * kMockDataLoaderName{"mock_data_loader"};
  static constexpr std::size_t kTimeStampsNumber{3};
  static const std::filesystem::path kMockLoadPath;
  static const r2k_core::Timestamps kTestTimestamps;

  class MockDataLoader final : public r2k_core::DataLoaderBase
  {
  public:
    explicit MockDataLoader(const std::string & name = std::string(kMockDataLoaderName))
    : r2k_core::DataLoaderBase(name)
    {
    }

  protected:
    bool setup_internal(
      const r2k_core::Timestamps & timestamps,
      [[maybe_unused]] const std::filesystem::path & load_path) final
    {
      timestamps_ = timestamps;
      ready_ = true;
      return true;
    }

    bool prepare_data_internal([[maybe_unused]] std::size_t idx) final { return true; }
  };
};
const std::filesystem::path TestDataLoaderBase::kMockLoadPath{
  std::filesystem::temp_directory_path()};
const r2k_core::Timestamps TestDataLoaderBase::kTestTimestamps(
  TestDataLoaderBase::kTimeStampsNumber);

TEST_F(TestDataLoaderBase, NameTest)
{
  const auto loader = MockDataLoader();
  ASSERT_STREQ(loader.name().c_str(), kMockDataLoaderName);
}

TEST_F(TestDataLoaderBase, SetupTest)
{
  auto loader = MockDataLoader();
  ASSERT_FALSE(loader.ready());
  ASSERT_EQ(loader.data_size(), std::size_t{0});
  ASSERT_TRUE(loader.setup(kTestTimestamps, kMockLoadPath));
  ASSERT_TRUE(loader.ready());
  ASSERT_EQ(loader.data_size(), kTestTimestamps.size());
  ASSERT_FALSE(loader.setup(r2k_core::Timestamps(4), kMockLoadPath));
  ASSERT_TRUE(loader.ready());
  ASSERT_EQ(loader.data_size(), kTestTimestamps.size());
}

TEST_F(TestDataLoaderBase, PrepareDataTest)
{
  auto loader = MockDataLoader();
  ASSERT_FALSE(loader.ready());
  ASSERT_FALSE(loader.prepare_data(0));
  ASSERT_TRUE(loader.setup(kTestTimestamps, kMockLoadPath));
  ASSERT_TRUE(loader.ready());
  for (std::size_t i = 0; i < kTimeStampsNumber; i++) {
    ASSERT_TRUE(loader.prepare_data(i));
  }
  ASSERT_FALSE(loader.prepare_data(kTimeStampsNumber));
};
