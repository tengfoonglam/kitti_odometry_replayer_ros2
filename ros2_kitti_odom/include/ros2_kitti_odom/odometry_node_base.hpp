#ifndef ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
#define ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace r2k_odom
{

class OdometryNodeBase : public rclcpp::Node
{
public:
  explicit OdometryNodeBase(const rclcpp::NodeOptions & options);
};

}  // namespace r2k_odom

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
