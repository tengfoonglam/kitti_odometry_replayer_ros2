#ifndef ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_
#define ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_

#include <kiss_icp/pipeline/KissICP.hpp>
#include <memory>
#include <mutex>
#include <ros2_kitti_core/timer.hpp>
#include <ros2_kitti_odom/odometry_node_base.hpp>
#include <string>

namespace r2k_odom_kiss_icp
{

class KissICPOdometryNode final : public r2k_odom::OdometryNodeBase
{
public:
  static constexpr std::size_t kLoggingPeriodMs = 10000;
  static constexpr double kSecondsToMsScalingFactor = 1e3;

  explicit KissICPOdometryNode(const rclcpp::NodeOptions & options);

private:
  std::mutex mutex_;
  std::string config_path;
  std::unique_ptr<kiss_icp::pipeline::KissICP> odometry_ptr_;
  kiss_icp::pipeline::KISSConfig config_;
  r2k_core::Timer timer_;

  void point_cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_ptr);
  bool reset_internal() final;
};

}  // namespace r2k_odom_kiss_icp

#endif  // ROS2_KITTI_ODOM_KISS_ICP__KISS_ICP_ODOMETRY_NODE_HPP_
