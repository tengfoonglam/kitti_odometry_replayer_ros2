#ifndef ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_

#include <open3d/Open3D.h>
#include <tf2/LinearMath/Transform.h>

#include <kiss_icp/pipeline/KissICP.hpp>
#include <memory>
#include <mutex>
#include <ros2_kitti_odom/odometry_node_base.hpp>
#include <string>

namespace r2k_odom_kiss_icp
{

class KissICPOdometryNode final : public r2k_odom::OdometryNodeBase
{
public:
  static constexpr std::size_t kLoggingPeriodMs = 10000;

  explicit KissICPOdometryNode(const rclcpp::NodeOptions & options);

private:
  std::mutex mutex_;
  std::string config_path;
  std::unique_ptr<kiss_icp::pipeline::KissICP> odometry_ptr_;
  kiss_icp::pipeline::KISSConfig config_;

  void point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr) final;
  bool reset_internal() final;
};

}  // namespace r2k_odom_kiss_icp

#endif  // ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_
