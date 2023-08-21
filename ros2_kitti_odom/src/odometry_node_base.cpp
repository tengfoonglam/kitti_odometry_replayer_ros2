#include "ros2_kitti_odom/odometry_node_base.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace r2k_odom
{

const tf2::Transform OdometryNodeBase::kIdentityTransform{tf2::Quaternion{0., 0., 0., 1.}};

[[nodiscard]] std::unique_ptr<OdometryNodeBase::ImageSubscriber>
create_image_subscriber_if_topic_not_empty(rclcpp::Node * node_ptr, const std::string & topic_name)
{
  const bool empty_topic_name = topic_name.empty();

  if (empty_topic_name) {
    return nullptr;
  } else {
    RCLCPP_INFO(
      node_ptr->get_logger(), "Initialized image subscriber with topic %s", topic_name.c_str());
    return std::make_unique<OdometryNodeBase::ImageSubscriber>(
      node_ptr, topic_name, std::string{OdometryNodeBase::kImageTransportHint});
  }
}

OdometryNodeBase::OdometryNodeBase(const rclcpp::NodeOptions & options)
: Node("kitti_odometry", options),
  odom_tf_sensor_(kIdentityTransform),
  sensor_tf_base_link_(kIdentityTransform),
  steady_clock_(RCL_STEADY_TIME)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  declare_parameter("odometry_frame_id", kDefaultOdometryFrameId);
  declare_parameter("pointcloud_topic", "");
  declare_parameter("p0_img_topic", "");
  declare_parameter("p1_img_topic", "");
  declare_parameter("p2_img_topic", "");
  declare_parameter("p3_img_topic", "");
  declare_parameter("base_link_frame_id", "");
  declare_parameter("sensor_frame_id", "");
  declare_parameter("config_path", "");

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
    parameters_client.get_parameter("pointcloud_topic", std::string{""});
  const auto p0_topic = parameters_client.get_parameter("p0_img_topic", std::string{""});
  const auto p1_topic = parameters_client.get_parameter("p1_img_topic", std::string{""});
  const auto p2_topic = parameters_client.get_parameter("p2_img_topic", std::string{""});
  const auto p3_topic = parameters_client.get_parameter("p3_img_topic", std::string{""});
  base_link_frame_id_ = parameters_client.get_parameter("base_link_frame_id", std::string{""});
  sensor_frame_id_ = parameters_client.get_parameter("sensor_frame_id", std::string{""});
  config_path_ = parameters_client.get_parameter("config_path", std::string{""});

  shutdown_if_empty(odometry_frame_id_, "odometry_frame_id");
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
      const auto base_link_tf_sensor_stamped = tf_listener_buffer_ptr->lookupTransform(
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
  tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  path_pub_ptr_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

  if (!pointcloud_topic.empty()) {
    point_cloud_sub_ptr_ = std::make_unique<PointCloudSubscriber>(this, pointcloud_topic);
    RCLCPP_INFO(
      get_logger(), "Initialized point cloud subscriber with topic %s", pointcloud_topic.c_str());
  }

  p0_img_sub_ptr_ = create_image_subscriber_if_topic_not_empty(this, p0_topic);
  p1_img_sub_ptr_ = create_image_subscriber_if_topic_not_empty(this, p1_topic);
  p2_img_sub_ptr_ = create_image_subscriber_if_topic_not_empty(this, p2_topic);
  p3_img_sub_ptr_ = create_image_subscriber_if_topic_not_empty(this, p3_topic);
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

}  // namespace r2k_odom
