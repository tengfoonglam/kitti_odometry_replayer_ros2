#include "ros2_kitti_core/point_cloud_data_loader.hpp"

namespace r2k_core
{

PointCloudDataLoader::PointCloudDataLoader(const std::string & name, const Header & header)
: FolderDataLoader<PointCloudDataLoader::ReturnType>(name, header)
{
}

PointCloudDataLoader::PointCloudDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header)
: FolderDataLoader<PointCloudDataLoader::ReturnType>(name, logger, header)
{
}

[[nodiscard]] std::optional<size_t> PointCloudDataLoader::get_last_index_of_sequence(
  const std::filesystem::path & load_path)
{
  return get_last_index_of_point_cloud_sequence(load_path);
}

[[nodiscard]] PointCloudDataLoader::ReturnType PointCloudDataLoader::load_data(
  const std::size_t idx, const std::filesystem::path & load_path)
{
  return load_point_cloud_from_file(from_index_to_point_cloud_file_path(idx, load_path));
}

}  // namespace r2k_core
