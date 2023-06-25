#include "ros2_kitti_odom/odometry_node_base.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace r2k_odom
{

const tf2::Transform OdometryNodeBase::kIdentityTransform{tf2::Quaternion{0., 0., 0., 1.}};

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options),
  odom_tf_sensor_(kIdentityTransform),
  sensor_tf_base_link_(kIdentityTransform)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  declare_parameter("odometry_frame_id", kDefaultOdometryFrameId);
  declare_parameter("pointcloud_topic", kDefaultPointCloudTopicName);
  declare_parameter("base_link_frame_id", "");
  declare_parameter("sensor_frame_id", "");

  // Wait for parameters to be loaded
  auto parameters_client = rclcpp::SyncParametersClient(this);
  while (!parameters_client.wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the parameters service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Parameters service not available, waiting again...");
  }

  odometry_frame_id_ =
    parameters_client.get_parameter("odometry_frame_id", std::string{kDefaultOdometryFrameId});
  const auto pointcloud_topic =
    parameters_client.get_parameter("pointcloud_topic", std::string{kDefaultPointCloudTopicName});
  base_link_frame_id_ = parameters_client.get_parameter("base_link_frame_id", std::string{""});
  sensor_frame_id_ = parameters_client.get_parameter("sensor_frame_id", std::string{""});

  shutdown_if_empty(odometry_frame_id_, "odometry_frame_id");
  shutdown_if_empty(pointcloud_topic, "pointcloud_topic");
  shutdown_if_empty(base_link_frame_id_, "base_link_frame_id");
  shutdown_if_empty(sensor_frame_id_, "sensor_frame_id");

  // Look up base-link -> sensor frame transform
  if (base_link_frame_id_ != sensor_frame_id_) {
    const auto tf_listener_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    const auto tf_listener_ptr_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_listener_buffer_ptr);

    const auto from_frame = sensor_frame_id_;
    const auto to_frame = base_link_frame_id_;

    RCLCPP_INFO(
      get_logger(), "Looking up TF from %s (sensor) to %s (base link)", from_frame.c_str(),
      to_frame.c_str());
    try {
      auto base_link_tf_sensor_stamped = tf_listener_buffer_ptr->lookupTransform(
        to_frame, from_frame, tf2::TimePointZero,
        tf2::durationFromSec(kBaseLinkTFScannerLookupTimeout));
      tf2::Transform base_link_tf_sensor;
      tf2::fromMsg(base_link_tf_sensor_stamped.transform, base_link_tf_sensor);
      // At initialization, base link should be aligned with the odom frame
      odom_tf_sensor_ = base_link_tf_sensor;
      sensor_tf_base_link_ = base_link_tf_sensor.inverse();
      RCLCPP_INFO(
        get_logger(), "Sucessfully established TF from %s (sensor) to %s (base link)",
        from_frame.c_str(), to_frame.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Could not find TF from %s to %s: %s. Exiting odometry node.",
        from_frame.c_str(), to_frame.c_str(), ex.what());
      rclcpp::shutdown();
    }
  }

  // Setup services, subcribers and publishers
  reset_service_ptr_ = create_service<TriggerSrv>(
    "~/reset",
    std::bind(&OdometryNodeBase::reset, this, std::placeholders::_1, std::placeholders::_2));
  set_transform_service_ptr_ = create_service<SetCurrentTransformSrv>(
    "~/set_current_transform", std::bind(
                                 &OdometryNodeBase::set_current_transform, this,
                                 std::placeholders::_1, std::placeholders::_2));
  tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  path_pub_ptr_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
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

void OdometryNodeBase::notify_new_transform(
  const Time & timestamp, const tf2::Transform & sensor_start_tf_sensor_current)
{
  std::scoped_lock lock(path_mutex_);

  const auto odom_tf_base_link_current =
    odom_tf_sensor_ * sensor_start_tf_sensor_current * sensor_tf_base_link_;

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = odometry_frame_id_;
  transform_stamped.header.stamp = timestamp;
  transform_stamped.child_frame_id = base_link_frame_id_;
  tf2::toMsg(odom_tf_base_link_current, transform_stamped.transform);
  tf_broadcaster_ptr_->sendTransform(transform_stamped);

  path_.header = transform_stamped.header;
  path_.poses.emplace_back();
  auto & new_pose_stamped = path_.poses.back();
  geometry_msgs::msg::PoseStamped pose_stamped;
  new_pose_stamped.header = transform_stamped.header;
  new_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
  new_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
  new_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
  new_pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;
  new_pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
  new_pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
  new_pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;

  path_pub_ptr_->publish(path_);
}

void OdometryNodeBase::set_current_transform(
  [[maybe_unused]] const std::shared_ptr<SetCurrentTransformSrv::Request> request_ptr,
  std::shared_ptr<SetCurrentTransformSrv::Response> response_ptr)
{
  response_ptr->response.success = set_current_transform_internal(request_ptr->request);
}

}  // namespace r2k_odom
