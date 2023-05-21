#include "ros2_kitti_odom/odometry_node_base.hpp"

namespace r2k_odom
{

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  reset_service_ptr_ = create_service<TriggerSrv>(
    "~/reset",
    std::bind(&OdometryNodeBase::reset, this, std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odometry_pub_ptr_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

  point_cloud_sub_ptr_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "point_cloud", 10, std::bind(&OdometryNodeBase::point_cloud_cb, this, std::placeholders::_1));
}

void OdometryNodeBase::reset(
  [[maybe_unused]] const std::shared_ptr<TriggerSrv::Request> request_ptr,
  std::shared_ptr<TriggerSrv::Response> response_ptr)
{
  response_ptr->success = reset_internal();
}

void OdometryNodeBase::point_cloud_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_ptr)
{
  point_cloud_cb_internal(pc_ptr);
}

void OdometryNodeBase::notify_new_transform(
  [[maybe_unused]] geometry_msgs::msg::TransformStamped transform)
{
}

}  // namespace r2k_odom
