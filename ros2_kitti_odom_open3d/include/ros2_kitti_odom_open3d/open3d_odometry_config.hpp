#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_

#include <open3d/Open3D.h>

#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace r2k_odom_o3d
{
struct ICPIterationSettings
{
  double max_correspondence_distance;
  open3d::pipelines::registration::ICPConvergenceCriteria convergence_criteria;

  /**
   * @brief Construct a new ICPIterationSettings object
   *        Refer to Open3D's open3d::pipelines::registration::RegistrationICP documentation for
   *        explanation of settings
   * @param max_correspondence_distance_in
   * @param relative_fitness
   * @param relative_rmse
   * @param max_iteration
   */
  ICPIterationSettings(
    double max_correspondence_distance_in = 1.0, double relative_fitness = 1e-6,
    double relative_rmse = 1e-6, int max_iteration = 30);

  friend std::ostream & operator<<(std::ostream & os, const ICPIterationSettings & setting);
};

struct NormalComputationSettings
{
  bool fast_normal_computation;
  int max_nn;
  double radius;

  /**
   * @brief Construct a new Normal Computation Settings object
   *        Refer to Open3D's open3d::geometry::KDTreeSearchParamHybrid documentation for
   *        explanation of settings
   * @param fast_normal_computation_in
   * @param max_nn_in
   * @param radius_in
   */
  NormalComputationSettings(
    bool fast_normal_computation_in = true, int max_nn_in = 30, double radius_in = 1.0);

  friend std::ostream & operator<<(std::ostream & os, const NormalComputationSettings & setting);
};

struct O3DICPConfig
{
  float decimation_factor = 0.1;
  NormalComputationSettings normal_computation;
  std::vector<ICPIterationSettings> iterations;

  /**
   * @brief Construct a new O3D ICPConfig object
   *
   * @param decimation_factor_in - Fraction of point clound to subsample before running ICP between
   *                               values 0.0-1.0
   * @param normal_computation_in - Normal compuation settings
   * @param iterations_in - ICP settings, length of vector is number of ICP operations to perfom
   */
  O3DICPConfig(
    float decimation_factor_in = 0.1,
    const NormalComputationSettings & normal_computation_in = NormalComputationSettings(),
    const std::vector<ICPIterationSettings> & iterations_in = {
      {1.5, 1e-4, 1e-4, 30}, {0.5, 1e-5, 1e-5, 20}, {0.05, 1e-6, 1e-6, 15}});

  friend std::ostream & operator<<(std::ostream & os, const O3DICPConfig & config);
};

std::optional<O3DICPConfig> load_config(const std::string & file_path_str);

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_CONFIG_HPP_
