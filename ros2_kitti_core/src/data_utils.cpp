#include "ros2_kitti_core/data_utils.hpp"

namespace r2k_core
{

[[nodiscard]] bool file_exists_and_correct_extension(
  const std::filesystem::path & path, const std::string & extension)
{
  return (std::filesystem::exists(path) && path.extension().string() == std::string{extension});
}

}  // namespace r2k_core
