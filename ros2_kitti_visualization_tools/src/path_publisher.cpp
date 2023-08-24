#include "ros2_kitti_visualization_tools/path_publisher.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace r2k_viz_tools
{

PathPublisher::PathPublisher(
  rclcpp::Node * const node_ptr, const std::string & base_frame_id, const std::string & topic_name)
: visual_tools_ptr_(std::make_shared<decltype(visual_tools_ptr_)::element_type>(
    base_frame_id, topic_name, node_ptr))
{
}

void PathPublisher::publish(const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  std::scoped_lock lock(mutex_);
  if (previous_transform_stamped_.has_value()) {
    const auto & previous_trans = previous_transform_stamped_.value().transform.translation;
    const auto & current_trans = transform_stamped.transform.translation;
    visual_tools_ptr_->publishLine(
      {previous_trans.x, previous_trans.y, previous_trans.z},
      {current_trans.x, current_trans.y, current_trans.z});
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = transform_stamped.transform.translation.x;
  pose.position.y = transform_stamped.transform.translation.y;
  pose.position.z = transform_stamped.transform.translation.z;
  pose.orientation.w = transform_stamped.transform.rotation.w;
  pose.orientation.x = transform_stamped.transform.rotation.x;
  pose.orientation.y = transform_stamped.transform.rotation.y;
  pose.orientation.z = transform_stamped.transform.rotation.z;
  visual_tools_ptr_->publishAxis(pose, kAxisLength, kAxisRadius);

  visual_tools_ptr_->trigger();
}

void PathPublisher::reset()
{
  std::scoped_lock lock(mutex_);
  visual_tools_ptr_->deleteAllMarkers();
  previous_transform_stamped_ = std::nullopt;
}

}  // namespace r2k_viz_tools
