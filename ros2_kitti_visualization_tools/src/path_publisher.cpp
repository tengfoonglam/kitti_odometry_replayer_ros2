#include "ros2_kitti_visualization_tools/path_publisher.hpp"

namespace r2k_viz_tools
{

PathPublisher::PathPublisher(rclcpp::Node * const node_ptr, const std::string & topic_name)
{
  publisher_ptr_ = node_ptr->create_publisher<MarkerArray>(topic_name, kPublisherHistoryDepth);
}

void PathPublisher::publish(const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  std::scoped_lock lock(mutex_);
  auto msg_ptr = std::make_unique<MarkerArray>();
  if (previous_transform_stamped_.has_value()) {
    msg_ptr->markers.push_back(
      get_line_marker(line_id_++, previous_transform_stamped_.value(), transform_stamped));
  }
  msg_ptr->markers.push_back(get_pose_marker(pose_id_++, transform_stamped));
  publisher_ptr_->publish(std::move(msg_ptr));
  previous_transform_stamped_ = transform_stamped;
}

PathPublisher::Marker PathPublisher::get_pose_marker(
  std::size_t id, const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  Marker msg;
  msg.header = transform_stamped.header;
  msg.ns = "pose";
  msg.id = id;
  msg.type = Marker::SPHERE;
  msg.action = Marker::ADD;
  msg.pose.position.x = transform_stamped.transform.translation.x;
  msg.pose.position.y = transform_stamped.transform.translation.y;
  msg.pose.position.z = transform_stamped.transform.translation.z;
  msg.pose.orientation.w = transform_stamped.transform.rotation.w;
  msg.pose.orientation.x = transform_stamped.transform.rotation.x;
  msg.pose.orientation.y = transform_stamped.transform.rotation.y;
  msg.pose.orientation.z = transform_stamped.transform.rotation.z;
  msg.scale.x = kPoseScale;
  msg.scale.y = kPoseScale;
  msg.scale.z = kPoseScale;
  msg.color.r = std::get<0>(kPoseRgba);
  msg.color.g = std::get<1>(kPoseRgba);
  msg.color.b = std::get<2>(kPoseRgba);
  msg.color.a = std::get<3>(kPoseRgba);
  return msg;
}

PathPublisher::Marker PathPublisher::get_line_marker(
  std::size_t id, const geometry_msgs::msg::TransformStamped & previous_transform_stamped,
  const geometry_msgs::msg::TransformStamped & current_transform_stamped)
{
  Marker msg;
  msg.header = current_transform_stamped.header;
  msg.ns = "line";
  msg.id = id;
  msg.type = Marker::LINE_STRIP;
  msg.action = Marker::ADD;
  msg.scale.x = kLineScale;
  msg.color.r = std::get<0>(kLineRgba);
  msg.color.g = std::get<1>(kLineRgba);
  msg.color.b = std::get<2>(kLineRgba);
  msg.color.a = std::get<3>(kLineRgba);

  using Point = Marker::_points_type::value_type;

  Point previous_point;
  const auto & previous_translation = previous_transform_stamped.transform.translation;
  previous_point.x = previous_translation.x;
  previous_point.y = previous_translation.y;
  previous_point.z = previous_translation.z;
  msg.points.push_back(previous_point);

  Point current_point;
  const auto & current_translation = current_transform_stamped.transform.translation;
  current_point.x = current_translation.x;
  current_point.y = current_translation.y;
  current_point.z = current_translation.z;
  msg.points.push_back(current_point);

  return msg;
}

void PathPublisher::reset()
{
  std::scoped_lock lock(mutex_);
  line_id_ = 0;
  pose_id_ = 0;
  previous_transform_stamped_ = std::nullopt;

  auto delete_all_msg_ptr = std::make_unique<MarkerArray>();
  Marker delete_msg;
  delete_msg.action = Marker::DELETEALL;
  delete_all_msg_ptr->markers.push_back(delete_msg);
  publisher_ptr_->publish(std::move(delete_all_msg_ptr));
}

}  // namespace r2k_viz_tools
