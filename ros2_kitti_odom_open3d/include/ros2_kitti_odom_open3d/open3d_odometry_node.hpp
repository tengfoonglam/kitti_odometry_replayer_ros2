#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_

#include <open3d/Open3D.h>
#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <mutex>
#include <ros2_kitti_odom/odometry_node_base.hpp>
#include <vector>

namespace r2k_odom_o3d
{

class Open3DOdometryNode final : public r2k_odom::OdometryNodeBase
{
public:
  using O3DPointCloud = open3d::geometry::PointCloud;

  struct ICPIterationSettings
  {
    double max_correspondence_distance;
    open3d::pipelines::registration::ICPConvergenceCriteria convergence_criteria;

    ICPIterationSettings(
      const double max_correspondence_distance_in = 1.0, const double relative_fitness = 1e-6,
      const double relative_rmse = 1e-6, const int max_iteration = 30)
    : max_correspondence_distance(max_correspondence_distance_in),
      convergence_criteria{relative_fitness, relative_rmse, max_iteration}
    {
    }
  };

  struct NormalComputationSettings
  {
    bool fast_normal_computation;
    int max_nn;
    double radius;

    NormalComputationSettings(
      const bool fast_normal_computation_in = true, const int max_nn_in = 30,
      const double radius_in = 1.0)
    : fast_normal_computation(fast_normal_computation_in), max_nn(max_nn_in), radius(radius_in)
    {
    }
  };

  struct O3DICPSettings
  {
    NormalComputationSettings normal_computation;
    std::vector<ICPIterationSettings> iterations;

    O3DICPSettings(
      const NormalComputationSettings & normal_computation_in = NormalComputationSettings(),
      const std::vector<ICPIterationSettings> & iterations_in =
        {{1.0, 1e-4, 1e-4, 30}, {0.5, 1e-5, 1e-5, 30}, {0.05, 1e-6, 1e-6, 30}})
    : normal_computation(normal_computation_in), iterations(iterations_in)
    {
    }
  };

  explicit Open3DOdometryNode(const rclcpp::NodeOptions & options);

  [[nodiscard]] static tf2::Transform eigen_to_transform(const Eigen::Matrix4d & tf_eigen);

private:
  std::mutex mutex_;
  std::unique_ptr<O3DPointCloud> buffer_pc_ptr_;
  tf2::Transform current_transform_;
  O3DICPSettings settings_;

  void point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr) final;
  bool set_current_transform_internal(const geometry_msgs::msg::Transform & transform_msg) final;
  bool reset_internal() final;

  [[nodiscard]] tf2::Transform perform_registration(
    const O3DPointCloud & source, const O3DPointCloud & target);
};

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
