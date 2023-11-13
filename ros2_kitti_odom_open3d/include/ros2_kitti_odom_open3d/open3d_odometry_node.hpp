#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_

#include <open3d/Open3D.h>
#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <mutex>
#include <ros2_kitti_core/timer.hpp>
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
  static constexpr double kSecondsToMsScalingFactor = 1e3;

  /**
   * @brief Construct a new Open 3 D Odometry Node object
   *
   * @param options
   */
  explicit Open3DOdometryNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Convert an Eigen matrix to tf2::Transform
   *
   * @param tf_eigen
   * @return tf2::Transform
   */
  [[nodiscard]] static tf2::Transform eigen_to_transform(const Eigen::Matrix4d & tf_eigen);

private:
  std::mutex mutex_;
  std::shared_ptr<O3DPointCloud> buffer_pc_ptr_;
  tf2::Transform sensor_start_tf_sensor_current_;
  O3DICPConfig config_;
  r2k_core::Timer normal_computation_timer_;
  r2k_core::Timer icp_timer_;

  /**
   * @brief Callback when a new point cloud is published
   *
   * @param pc_ptr
   */
  void point_cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_ptr);

  /**
   * @brief Reset of node
   *
   * @return true - Reset successful
   * @return false - Otherwise
   */
  bool reset_internal() final;

  /**
   * @brief Perform ICP resgistration
   *
   * @param source - Point cloud to register
   * @param target - Point cloud to register against
   * @return tf2::Transform - Resulting transformation of source w.r.t target
   */
  [[nodiscard]] tf2::Transform perform_registration(
    const O3DPointCloud & source, const O3DPointCloud & target);
};

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
