#include "ros2_kitti_odom/odometry_node_base.hpp"

namespace r2k_odom
{

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
}

}  // namespace r2k_odom
