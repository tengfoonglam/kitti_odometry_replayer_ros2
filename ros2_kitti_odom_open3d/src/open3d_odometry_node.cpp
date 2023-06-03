#include "ros2_kitti_odom_open3d/open3d_odometry_node.hpp"

#include <Eigen/Core>
#include <open3d_conversions/open3d_conversions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
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
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.header.stamp = pc_ptr->header.stamp;
  transform_stamped.child_frame_id = pc_ptr->header.frame_id;

  if (buffer_pc_ptr_) {
    const auto result = perform_registration(*current_pc_ptr, *buffer_pc_ptr_);
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

tf2::Transform Open3DOdometryNode::eigen_to_transform(const Eigen::Matrix4d & tf_eigen)
{
  Eigen::Affine3d tf_eigen_affine;
  tf_eigen_affine.matrix() = tf_eigen;
  const auto tf_msg = tf2::eigenToTransform(tf_eigen_affine).transform;
  tf2::Transform tf_tf2;
  tf2::fromMsg(tf_msg, tf_tf2);
  return tf_tf2;
}

tf2::Transform Open3DOdometryNode::perform_registration(
  const O3DPointCloud & source, const O3DPointCloud & target)
{
  Eigen::Matrix4d result_eigen = Eigen::Matrix4d::Identity();

  for (const auto & icp_setting : icp_iteration_settings_) {
    const auto registration_result = open3d::pipelines::registration::RegistrationICP(
      source, target, icp_setting.max_corresponence_distance, result_eigen,
      open3d::pipelines::registration::TransformationEstimationPointToPlane(),
      icp_setting.convergence_criteria);
    result_eigen = registration_result.transformation_;
  }

  return eigen_to_transform(result_eigen);
}

}  // namespace r2k_odom_o3d

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_odom_o3d::Open3DOdometryNode)
