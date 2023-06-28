#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_

#include <open3d/Open3D.h>

#include <vector>

namespace r2k_odom_o3d
{
struct ICPIterationSettings
{
  double max_correspondence_distance;
  open3d::pipelines::registration::ICPConvergenceCriteria convergence_criteria;

  ICPIterationSettings(
    const double max_correspondence_distance_in = 1.0, const double relative_fitness = 1e-6,
    const double relative_rmse = 1e-6, const int max_iteration = 30);
};

struct NormalComputationSettings
{
  bool fast_normal_computation;
  int max_nn;
  double radius;

  NormalComputationSettings(
    const bool fast_normal_computation_in = true, const int max_nn_in = 30,
    const double radius_in = 1.0);
};

struct O3DICPConfig
{
  float decimation_factor = 0.1;
  NormalComputationSettings normal_computation;
  std::vector<ICPIterationSettings> iterations;

  O3DICPConfig(
    const float decimation_factor_in = 0.1,
    const NormalComputationSettings & normal_computation_in = NormalComputationSettings(),
    const std::vector<ICPIterationSettings> & iterations_in = {
      {1.0, 1e-4, 1e-4, 30}, {0.5, 1e-5, 1e-5, 20}, {0.05, 1e-6, 1e-6, 15}});
};
}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_
