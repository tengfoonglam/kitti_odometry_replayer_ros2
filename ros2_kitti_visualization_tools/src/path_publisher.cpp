#include "ros2_kitti_visualization_tools/path_publisher.hpp"

#include <geometry_msgs/msg/polygon.hpp>

namespace r2k_viz_tools
{

PathPublisher::PathPublisher(
  rclcpp::Node * const node_ptr, const std::string & base_frame_id, const std::string & topic_name)
: visual_tools_ptr_(std::make_shared<decltype(visual_tools_ptr_)::element_type>(
    base_frame_id, topic_name, node_ptr))
{
}

void PathPublisher::publish(const TransformStamped & transform_stamped)
{
  std::scoped_lock lock(mutex_);
  const auto current_point = vector3_to_point(transform_stamped.transform.translation);
  if (previous_point_opt_.has_value()) {
    visual_tools_ptr_->publishPath(
      {previous_point_opt_.value(), current_point}, kPathColour, kPathRadius);
  }
  visual_tools_ptr_->publishAxis(
    transform_to_pose(transform_stamped.transform), kAxisLength, kAxisRadius);

  visual_tools_ptr_->trigger();
  previous_point_opt_ = current_point;
}

void PathPublisher::reset()
{
  std::scoped_lock lock(mutex_);
  visual_tools_ptr_->deleteAllMarkers();
  previous_point_opt_ = std::nullopt;
}

PathPublisher::Pose PathPublisher::transform_to_pose(const Transform & transform)
{
  Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation.w = transform.rotation.w;
  pose.orientation.x = transform.rotation.x;
  pose.orientation.y = transform.rotation.y;
  pose.orientation.z = transform.rotation.z;
  return pose;
}

PathPublisher::Point PathPublisher::vector3_to_point(const Vector3 & vector_3)
{
  Point point;
  point.x = vector_3.x;
  point.y = vector_3.y;
  point.z = vector_3.z;
  return point;
}

}  // namespace r2k_viz_tools
