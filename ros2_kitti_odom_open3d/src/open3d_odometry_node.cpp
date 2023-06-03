#include "ros2_kitti_odom_open3d/open3d_odometry_node.hpp"

#include <open3d_conversions/open3d_conversions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace r2k_odom_o3d
{

Open3DOdometryNode::Open3DOdometryNode(const rclcpp::NodeOptions & options)
: r2k_odom::OdometryNodeBase(options)
{
}

void Open3DOdometryNode::point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr)
{
  std::scoped_lock lock(mutex_);

  auto current_pc_ptr = std::make_unique<O3DPointCloud>();
  static constexpr bool kSkipColour = true;
  open3d_conversions::rosToOpen3d(pc_ptr, *current_pc_ptr, kSkipColour);

  // Compute normal
  const int max_nn = 30;
  const int radius = 1.0;
  const bool fast_normal_computation = true;
  current_pc_ptr->EstimateNormals(
    open3d::geometry::KDTreeSearchParamHybrid(radius, max_nn), fast_normal_computation);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header = pc_ptr->header;
  transform_stamped.child_frame_id = odom_child_id_;

  if (buffer_pc_ptr_) {
    const auto result = perform_registration(*buffer_pc_ptr_, *current_pc_ptr);
    current_transform_ *= result;
    transform_stamped.transform = tf2::toMsg(current_transform_);
  }

  // Notify results
  notify_new_transform(transform_stamped);

  // Update PC buffer
  buffer_pc_ptr_.swap(current_pc_ptr);
}

bool Open3DOdometryNode::reset_internal()
{
  std::scoped_lock lock(mutex_);
  current_transform_ = tf2::Transform();
  buffer_pc_ptr_.reset();
  return true;
}

tf2::Transform Open3DOdometryNode::perform_registration(
  const O3DPointCloud & source, const O3DPointCloud & target)
{
  O3DPointCloud target_cp{target};

  return tf2::Transform{};
}

}  // namespace r2k_odom_o3d

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_odom_o3d::Open3DOdometryNode)
