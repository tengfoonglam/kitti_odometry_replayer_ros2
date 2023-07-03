#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_

#include <open3d/Open3D.h>
#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <mutex>
#include <ros2_kitti_odom/odometry_node_base.hpp>
#include <vector>

#include "ros2_kitti_odom_open3d/open3d_odometry_config.hpp"

namespace r2k_odom_o3d
{

class Open3DOdometryNode final : public r2k_odom::OdometryNodeBase
{
public:
  using O3DPointCloud = open3d::geometry::PointCloud;
  static constexpr std::size_t kLoggingPeriodMs = 10000;

  explicit Open3DOdometryNode(const rclcpp::NodeOptions & options);

  [[nodiscard]] static tf2::Transform eigen_to_transform(const Eigen::Matrix4d & tf_eigen);

private:
  std::mutex mutex_;
  std::shared_ptr<O3DPointCloud> buffer_pc_ptr_;
  tf2::Transform sensor_start_tf_sensor_current_;
  O3DICPConfig config_;

  void point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr) final;
  bool reset_internal() final;

  [[nodiscard]] tf2::Transform perform_registration(
    const O3DPointCloud & source, const O3DPointCloud & target);
};

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
