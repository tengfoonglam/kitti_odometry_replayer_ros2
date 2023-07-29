#include "ros2_kitti_core/image_data_loader.hpp"

namespace r2k_core
{

ImageDataLoader::ImageDataLoader(const std::string & name, const Header & header)
: FolderDataLoader<ImageDataLoader::ReturnType>(name, header)
{
}

ImageDataLoader::ImageDataLoader(
  const std::string & name, rclcpp::Logger logger, const Header & header)
: FolderDataLoader<ImageDataLoader::ReturnType>(name, logger, header)
{
}

std::optional<size_t> ImageDataLoader::get_last_index_of_sequence(
  const std::filesystem::path & load_path)
{
  return get_last_index_of_image_sequence(load_path);
}

ImageDataLoader::ReturnType ImageDataLoader::load_data(
  std::size_t idx, const std::filesystem::path & load_path)
{
  return load_image_from_file(from_index_to_image_file_path(idx, load_path));
}

}  // namespace r2k_core
