#include "ros2_kitti_visualization_tools/path_publisher.hpp"

namespace r2k_viz_tools
{

PathPublisher::PathPublisher(rclcpp::Node::SharedPtr node_ptr, const std::string & topic_name)
{
  publisher_ptr_ = node_ptr->create_publisher<Marker>(topic_name, 10);
}

void PathPublisher::publish(const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  publish_line(transform_stamped);
  publish_pose(transform_stamped);
}

void PathPublisher::publish_pose(const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  std::scoped_lock lock(mutex_);
  auto msg_ptr = std::make_unique<Marker>();
  msg_ptr->header = transform_stamped.header;
  msg_ptr->ns = "pose";
  msg_ptr->id = pose_id_++;
  msg_ptr->type = Marker::SPHERE;
  msg_ptr->action = Marker::ADD;
  msg_ptr->pose.position.x = transform_stamped.transform.translation.x;
  msg_ptr->pose.position.y = transform_stamped.transform.translation.y;
  msg_ptr->pose.position.z = transform_stamped.transform.translation.z;
  msg_ptr->pose.orientation.w = transform_stamped.transform.rotation.w;
  msg_ptr->pose.orientation.x = transform_stamped.transform.rotation.x;
  msg_ptr->pose.orientation.y = transform_stamped.transform.rotation.y;
  msg_ptr->pose.orientation.z = transform_stamped.transform.rotation.z;
  msg_ptr->scale.x = kPoseScale;
  msg_ptr->scale.y = kPoseScale;
  msg_ptr->scale.z = kPoseScale;
  msg_ptr->color.r = std::get<0>(kPoseRgba);
  msg_ptr->color.g = std::get<1>(kPoseRgba);
  msg_ptr->color.b = std::get<2>(kPoseRgba);
  msg_ptr->color.a = std::get<3>(kPoseRgba);
  publisher_ptr_->publish(std::move(msg_ptr));
}

void PathPublisher::publish_line(const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  std::scoped_lock lock(mutex_);

  if (previous_transform_stamped_.has_value()) {
    auto msg_ptr = std::make_unique<Marker>();
    msg_ptr->header = transform_stamped.header;
    msg_ptr->ns = "line";
    msg_ptr->id = line_id_++;
    msg_ptr->type = Marker::LINE_STRIP;
    msg_ptr->action = Marker::ADD;
    msg_ptr->scale.x = kLineScale;
    msg_ptr->color.r = std::get<0>(kLineRgba);
    msg_ptr->color.g = std::get<1>(kLineRgba);
    msg_ptr->color.b = std::get<2>(kLineRgba);
    msg_ptr->color.a = std::get<3>(kLineRgba);

    using Point = Marker::_points_type::value_type;

    Point previous_point;
    const auto & previous_translation = previous_transform_stamped_.value().transform.translation;
    previous_point.x = previous_translation.x;
    previous_point.y = previous_translation.y;
    previous_point.z = previous_translation.z;
    msg_ptr->points.push_back(previous_point);

    Point current_point;
    const auto & current_translation = transform_stamped.transform.translation;
    current_point.x = current_translation.x;
    current_point.y = current_translation.y;
    current_point.z = current_translation.z;
    msg_ptr->points.push_back(previous_point);

    publisher_ptr_->publish(std::move(msg_ptr));
  }

  previous_transform_stamped_ = transform_stamped;
}

void PathPublisher::reset()
{
  std::scoped_lock lock(mutex_);
  line_id_ = 0;
  pose_id_ = 0;
  previous_transform_stamped_ = std::nullopt;

  auto delete_all_msg_ptr = std::make_unique<Marker>();
  delete_all_msg_ptr->action = Marker::DELETEALL;
  publisher_ptr_->publish(std::move(delete_all_msg_ptr));
}

}  // namespace r2k_viz_tools
