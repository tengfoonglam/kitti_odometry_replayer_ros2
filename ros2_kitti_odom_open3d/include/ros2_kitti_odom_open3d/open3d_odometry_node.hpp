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

  struct ICPSettings
  {
    double max_corresponence_distance = 1.0;
    open3d::pipelines::registration::ICPConvergenceCriteria convergence_criteria;
  };

  explicit Open3DOdometryNode(const rclcpp::NodeOptions & options);

private:
  std::mutex mutex_;
  std::unique_ptr<O3DPointCloud> buffer_pc_ptr_;
  tf2::Transform current_transform_;
  std::vector<ICPSettings> icp_iteration_settings_;

  void point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr) final;
  bool reset_internal() final;

  tf2::Transform perform_registration(const O3DPointCloud & source, const O3DPointCloud & target);
};

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
