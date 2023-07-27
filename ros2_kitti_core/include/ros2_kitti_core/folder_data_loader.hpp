#ifndef ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
#define ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_

#include <algorithm>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "ros2_kitti_core/data_loader.hpp"
#include "ros2_kitti_core/timestamp_utils.hpp"
#include "ros2_kitti_core/type_checks.hpp"

namespace r2k_core
{

template <typename RT>
class FolderDataLoader : public DataLoader<RT>
{
  static_assert(
    (is_shared_ptr<RT>::value || is_unique_ptr<RT>::value),
    "Currently FolderDataLoader only supports unique or shared pointer return types");

public:
  using Header = std_msgs::msg::Header;

  FolderDataLoader(const std::string & name, const Header & header);

  FolderDataLoader(const std::string & name, rclcpp::Logger logger, const Header & header);

  [[nodiscard]] std::size_t data_size() const final;

protected:
  Header header_;
  std::size_t max_idx_{0};
  RT data_ptr_;
  std::optional<size_t> current_idx_opt_;
  std::filesystem::path load_path_;

  [[nodiscard]] virtual std::optional<size_t> get_last_index_of_sequence(
    const std::filesystem::path & load_path) = 0;

  [[nodiscard]] virtual RT load_data(std::size_t idx, const std::filesystem::path & load_path) = 0;

  bool setup_internal(const Timestamps & timestamps, const std::filesystem::path & load_path) final;

  bool prepare_data_internal(std::size_t idx) final;

  std::optional<RT> get_data_internal(std::size_t idx) final;
};

}  // namespace r2k_core
#endif  // ROS2_KITTI_CORE__FOLDER_DATA_LOADER_HPP_
