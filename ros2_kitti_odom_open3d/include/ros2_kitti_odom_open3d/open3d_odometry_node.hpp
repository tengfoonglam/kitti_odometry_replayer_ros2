#ifndef ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_OPEN3D__OPEN3D_ODOMETRY_NODE_HPP_

#include <ros2_kitti_odom/odometry_node_base.hpp>

namespace r2k_odom_o3d
{

class Open3DOdometryNode final : public r2k_odom::OdometryNodeBase
{
  explicit Open3DOdometryNode(const rclcpp::NodeOptions & options);
};

}  // namespace r2k_odom_o3d

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
