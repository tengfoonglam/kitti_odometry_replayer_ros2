#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <iostream>
#include <ros2_kitti_core/point_cloud_utils.hpp>

int main(int argc, char const * argv[])
{
  std::string bin_to_load;

  if (argc == 2) {
    bin_to_load = static_cast<std::string>(argv[1]);
    std::cout << "Attempt to convert " << bin_to_load << std::endl;
  } else {
    std::cout << "Usage: convert_kitti_point_cloud path_to_bin_file" << std::endl;
    return 1;
  }

  const std::filesystem::path input_path{bin_to_load};
  auto msg_ptr = r2k_core::load_point_cloud_from_file(bin_to_load);

  if (!msg_ptr) {
    std::cerr << "Failed to load KITTI .bin point cloud from " << bin_to_load;
    return 1;
  }

  r2k_core::PointCloudPCLType pcl_cloud;
  pcl::fromROSMsg(*msg_ptr, pcl_cloud);

  std::filesystem::path output_path{input_path};
  output_path.replace_extension(".pcd");
  std::cout << "Saving point cloud with " << pcl_cloud.size() << " points to " << output_path
            << std::endl;

  try {
    pcl::io::savePCDFileASCII(output_path.string(), pcl_cloud);
  } catch (const std::exception & e) {
    std::cerr << "Saving " << output_path << " file failed. Error: " << e.what();
    return 1;
  }

  return 0;
}
