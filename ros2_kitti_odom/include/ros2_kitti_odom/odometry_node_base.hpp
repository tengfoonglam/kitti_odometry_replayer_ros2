#ifndef ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
#define ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_

#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace r2k_odom
{

class OdometryNodeBase : public rclcpp::Node
{
public:
  static constexpr const char * const kDefaultOdometryFrameId{"odom"};
  static constexpr const char * const kDefaultPointCloudTopicName{"odom_pointcloud"};
  static constexpr float kBaseLinkTFScannerLookupTimeout = 5.0;
  static const tf2::Transform kIdentityTransform;

  using Time = builtin_interfaces::msg::Time;
  using TriggerSrv = std_srvs::srv::Trigger;

  template <typename T>
  using Publisher = rclcpp::Publisher<T>;
  template <typename T>
  using Service = rclcpp::Service<T>;
  template <typename T>
  using Subscriber = message_filters::Subscriber<T>;

  explicit OdometryNodeBase(const rclcpp::NodeOptions & options);

  virtual ~OdometryNodeBase() = default;

protected:
  void reset(
    [[maybe_unused]] const std::shared_ptr<TriggerSrv::Request> request_ptr,
    std::shared_ptr<TriggerSrv::Response> response_ptr);

  virtual bool reset_internal()
  {
    std::scoped_lock lock(path_mutex_);
    path_ = nav_msgs::msg::Path();
    return true;
  }
  virtual void notify_new_transform(
    const Time & timestamp, const tf2::Transform & sensor_start_tf_sensor_current);

  void shutdown_if_empty(const std::string & string_to_check, const std::string & param_name);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
  Service<TriggerSrv>::SharedPtr reset_service_ptr_;
  std::shared_ptr<Publisher<nav_msgs::msg::Path>> path_pub_ptr_;
  std::unique_ptr<Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_sub_ptr_;
  std::string odometry_frame_id_;
  std::string base_link_frame_id_;
  std::string sensor_frame_id_;
  std::string config_path_;
  tf2::Transform odom_tf_sensor_;
  tf2::Transform sensor_tf_base_link_;
  nav_msgs::msg::Path path_;
  std::mutex path_mutex_;
  rclcpp::Clock steady_clock_;
};

}  // namespace r2k_odom

#endif  // ROS2_KITTI_ODOM__ODOMETRY_NODE_BASE_HPP_
