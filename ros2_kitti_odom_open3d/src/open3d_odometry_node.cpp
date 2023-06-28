#include "ros2_kitti_odom_open3d/open3d_odometry_node.hpp"

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <open3d_conversions/open3d_conversions.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace r2k_odom_o3d
{

Open3DOdometryNode::Open3DOdometryNode(const rclcpp::NodeOptions & options)
: r2k_odom::OdometryNodeBase(options), sensor_start_tf_sensor_current_(kIdentityTransform)
{
}

void Open3DOdometryNode::point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr)
{
  std::scoped_lock lock(mutex_);

  // Convert to Open3D Point Cloud
  auto current_pc_ptr = std::make_shared<O3DPointCloud>();
  static constexpr bool kSkipColour = true;
  open3d_conversions::rosToOpen3d(pc_ptr, *current_pc_ptr, kSkipColour);

  // Decimate point cloud and compute normals
  const auto normal_computation_start_time = steady_clock_.now();
  const auto & norm_settings = config_.normal_computation;

  if (config_.decimation_factor > 0.0 && config_.decimation_factor < 1.0) {
    current_pc_ptr = current_pc_ptr->RandomDownSample(config_.decimation_factor);
  }

  current_pc_ptr->EstimateNormals(
    open3d::geometry::KDTreeSearchParamHybrid(norm_settings.radius, norm_settings.max_nn),
    norm_settings.fast_normal_computation);
  const auto normal_computation_end_time = steady_clock_.now();
  const auto normal_computation_duration =
    normal_computation_end_time - normal_computation_start_time;
  RCLCPP_INFO_THROTTLE(
    get_logger(), steady_clock_, kLoggingPeriodMs,
    "Current decimation + normal computation time [ms]: %f",
    normal_computation_duration.seconds() * 1e3);

  // Perform registration in the event that there is a previous point cloud
  if (buffer_pc_ptr_) {
    const auto registration_start_time = steady_clock_.now();

    const auto result = perform_registration(*current_pc_ptr, *buffer_pc_ptr_);

    const auto resgistration_end_time = steady_clock_.now();
    const auto registration_duration = resgistration_end_time - registration_start_time;
    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, kLoggingPeriodMs, "Current registration time [ms]: %f",
      registration_duration.seconds() * 1e3);

    sensor_start_tf_sensor_current_ *= result;
  }

  // Notify results
  notify_new_transform(pc_ptr->header.stamp, sensor_start_tf_sensor_current_);

  // Update PC buffer
  buffer_pc_ptr_.swap(current_pc_ptr);
}

bool Open3DOdometryNode::reset_internal()
{
  std::scoped_lock lock(mutex_);
  sensor_start_tf_sensor_current_ = kIdentityTransform;
  buffer_pc_ptr_.reset();
  return r2k_odom::OdometryNodeBase::reset_internal();
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
  for (const auto & icp_setting : config_.iterations) {
    const auto registration_result = open3d::pipelines::registration::RegistrationICP(
      source, target, icp_setting.max_correspondence_distance, result_eigen,
      open3d::pipelines::registration::TransformationEstimationPointToPlane(),
      icp_setting.convergence_criteria);
    result_eigen = registration_result.transformation_;
  }
  return eigen_to_transform(result_eigen);
}

bool Open3DOdometryNode::set_current_transform_internal(
  const geometry_msgs::msg::Transform & transform_msg)
{
  reset_internal();
  {
    std::scoped_lock lock(mutex_);
    tf2::fromMsg(transform_msg, sensor_start_tf_sensor_current_);
  }
  return true;
}

}  // namespace r2k_odom_o3d

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_odom_o3d::Open3DOdometryNode)
