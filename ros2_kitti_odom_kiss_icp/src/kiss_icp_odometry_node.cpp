#include "ros2_kitti_odom_kiss_icp/kiss_icp_odometry_node.hpp"

#include <tf2/LinearMath/Transform.h>

#include <vector>

#include "ros2_kitti_odom_kiss_icp/utils.hpp"

namespace r2k_odom_kiss_icp
{

KissICPOdometryNode::KissICPOdometryNode(const rclcpp::NodeOptions & options)
: r2k_odom::OdometryNodeBase(options)
{
  std::scoped_lock lock(mutex_);

  if (!config_path_.empty()) {
    if (const auto config = load_config(config_path_); config.has_value()) {
      config_ = config.value();
      RCLCPP_INFO(get_logger(), "Successfully loaded config from: %s", config_path_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to load config from: %s. Using default settings.",
        config_path_.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "No config path provided. Using default settings.");
  }

  RCLCPP_INFO_STREAM(get_logger(), "Loaded settings: \n" << config_);

  odometry_ptr_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
}

void KissICPOdometryNode::point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr)
{
  std::scoped_lock lock(mutex_);

  const auto points = point_cloud_2_to_eigen(*pc_ptr);
  const auto timestamps = !config_.deskew ? std::vector<double>{} : get_timestamps(*pc_ptr);

  odometry_ptr_->RegisterFrame(points, timestamps);

  const auto pose = odometry_ptr_->poses().back();
  const Eigen::Vector3d trans = pose.translation();
  const Eigen::Quaterniond rot = pose.unit_quaternion();
  tf2::Transform sensor_start_tf_sensor_current(
    {rot.x(), rot.y(), rot.z(), rot.w()}, {trans.x(), trans.y(), trans.z()});

  notify_new_transform(pc_ptr->header.stamp, sensor_start_tf_sensor_current);
}

bool KissICPOdometryNode::reset_internal()
{
  std::scoped_lock lock(mutex_);
  odometry_ptr_ = std::make_unique<kiss_icp::pipeline::KissICP>(config_);
  return r2k_odom::OdometryNodeBase::reset_internal();
}

}  // namespace r2k_odom_kiss_icp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_odom_kiss_icp::KissICPOdometryNode)
