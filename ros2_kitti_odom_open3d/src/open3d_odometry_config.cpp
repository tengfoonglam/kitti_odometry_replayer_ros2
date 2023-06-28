#include "ros2_kitti_odom_open3d/open3d_odometry_config.hpp"

namespace r2k_odom_o3d
{

ICPIterationSettings::ICPIterationSettings(
  const double max_correspondence_distance_in, const double relative_fitness,
  const double relative_rmse, const int max_iteration)
: max_correspondence_distance(max_correspondence_distance_in),
  convergence_criteria{relative_fitness, relative_rmse, max_iteration}
{
}

NormalComputationSettings::NormalComputationSettings(
  const bool fast_normal_computation_in, const int max_nn_in, const double radius_in)
: fast_normal_computation(fast_normal_computation_in), max_nn(max_nn_in), radius(radius_in)
{
}

O3DICPConfig::O3DICPConfig(
  const float decimation_factor_in, const NormalComputationSettings & normal_computation_in,
  const std::vector<ICPIterationSettings> & iterations_in)
: decimation_factor(decimation_factor_in),
  normal_computation(normal_computation_in),
  iterations(iterations_in)
{
}

}  // namespace r2k_odom_o3d
