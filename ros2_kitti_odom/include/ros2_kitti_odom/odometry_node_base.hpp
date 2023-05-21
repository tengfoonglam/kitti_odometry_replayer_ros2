#ifndef ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
#define ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace r2k_odom
{

class OdometryNodeBase : public rclcpp::Node
{
public:
  using TriggerSrv = std_srvs::srv::Trigger;

  template <typename T>
  using Publisher = rclcpp::Publisher<T>;
  template <typename T>
  using Service = rclcpp::Service<T>;
  template <typename T>
  using Subscription = rclcpp::Subscription<T>;

  explicit OdometryNodeBase(const rclcpp::NodeOptions & options);

protected:
  void reset(
    [[maybe_unused]] const std::shared_ptr<TriggerSrv::Request> request_ptr,
    std::shared_ptr<TriggerSrv::Response> response_ptr);
  void point_cloud_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_ptr);

  virtual bool reset_internal() { return true; }
  virtual void notify_new_transform(
    [[maybe_unused]] geometry_msgs::msg::TransformStamped transform);
  virtual void point_cloud_cb_internal(
    [[maybe_unused]] sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_ptr)
  {
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
  Service<TriggerSrv>::SharedPtr reset_service_ptr_;
  std::shared_ptr<Publisher<nav_msgs::msg::Odometry>> odometry_pub_ptr_;
  std::shared_ptr<Subscription<sensor_msgs::msg::PointCloud2>> point_cloud_sub_ptr_;
};

}  // namespace r2k_odom

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
