#include "ros2_kitti_odom/odometry_node_base.hpp"

namespace r2k_odom
{

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  declare_parameter("pointcloud_topic", "");
  declare_parameter("odom_child_id", kDefaultOdomChildId);

  // Wait for parameters to be loaded
  auto parameters_client = rclcpp::SyncParametersClient(this);
  while (!parameters_client.wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the parameters service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Parameters service not available, waiting again...");
  }
  const auto pointcloud_topic = parameters_client.get_parameter("pointcloud_topic", std::string{});
  odom_child_id_ =
    parameters_client.get_parameter("odom_child_id", std::string{kDefaultOdomChildId});

  // Setup services, subcribers and publishers
  reset_service_ptr_ = create_service<TriggerSrv>(
    "~/reset",
    std::bind(&OdometryNodeBase::reset, this, std::placeholders::_1, std::placeholders::_2));
  tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odometry_pub_ptr_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);

  if (!pointcloud_topic.empty()) {
    point_cloud_sub_ptr_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&OdometryNodeBase::point_cloud_cb, this, std::placeholders::_1));
  }
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
