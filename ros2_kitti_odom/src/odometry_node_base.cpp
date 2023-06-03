#include "ros2_kitti_odom/odometry_node_base.hpp"

namespace r2k_odom
{

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  declare_parameter("global_frame_id", kDefaultGlobalFrameId);
  declare_parameter("pointcloud_topic", kDefaultPointCloudTopicName);

  // Wait for parameters to be loaded
  auto parameters_client = rclcpp::SyncParametersClient(this);
  while (!parameters_client.wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the parameters service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Parameters service not available, waiting again...");
  }

  global_frame_id_ =
    parameters_client.get_parameter("global_frame_id", std::string{kDefaultGlobalFrameId});
  const auto pointcloud_topic =
    parameters_client.get_parameter("pointcloud_topic", std::string{kDefaultPointCloudTopicName});

  shutdown_if_empty(global_frame_id_, "global_frame_id");
  shutdown_if_empty(pointcloud_topic, "pointcloud_topic");

  // Setup services, subcribers and publishers
  reset_service_ptr_ = create_service<TriggerSrv>(
    "~/reset",
    std::bind(&OdometryNodeBase::reset, this, std::placeholders::_1, std::placeholders::_2));
  tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odometry_pub_ptr_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
  point_cloud_sub_ptr_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, 10,
    std::bind(&OdometryNodeBase::point_cloud_cb, this, std::placeholders::_1));
}

void OdometryNodeBase::shutdown_if_empty(
  const std::string & string_to_check, const std::string & param_name)
{
  if (string_to_check.empty()) {
    RCLCPP_ERROR(get_logger(), "Empty %s param provided. Exiting.", param_name.c_str());
    rclcpp::shutdown();
  }
}

void OdometryNodeBase::reset(
  [[maybe_unused]] const std::shared_ptr<TriggerSrv::Request> request_ptr,
  std::shared_ptr<TriggerSrv::Response> response_ptr)
{
  response_ptr->success = reset_internal();
}

void OdometryNodeBase::point_cloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr)
{
  point_cloud_cb_internal(pc_ptr);
}

void OdometryNodeBase::notify_new_transform(geometry_msgs::msg::TransformStamped transform_stamped)
{
  tf_broadcaster_ptr_->sendTransform(transform_stamped);

  auto odom_msg_ptr = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg_ptr->header = transform_stamped.header;
  odom_msg_ptr->pose.pose.position.x = transform_stamped.transform.translation.x;
  odom_msg_ptr->pose.pose.position.y = transform_stamped.transform.translation.y;
  odom_msg_ptr->pose.pose.position.z = transform_stamped.transform.translation.z;
  odom_msg_ptr->pose.pose.orientation.w = transform_stamped.transform.rotation.w;
  odom_msg_ptr->pose.pose.orientation.x = transform_stamped.transform.rotation.x;
  odom_msg_ptr->pose.pose.orientation.y = transform_stamped.transform.rotation.y;
  odom_msg_ptr->pose.pose.orientation.z = transform_stamped.transform.rotation.z;
  odometry_pub_ptr_->publish(std::move(odom_msg_ptr));
}

}  // namespace r2k_odom
