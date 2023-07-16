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
  std::scoped_lock lock(mutex_);

  if (!config_path_.empty()) {
    if (const auto settings_opt = load_config(config_path_); settings_opt.has_value()) {
      config_ = settings_opt.value();
      RCLCPP_INFO(get_logger(), "Successfully loaded config from: %s", config_path_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to load config from: %s. Using default settings.",
        config_path_.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "No config path provided. Using default settings.");
  }

  RCLCPP_INFO_STREAM(get_logger(), "Loaded settings: \n" << config_);
}

void Open3DOdometryNode::point_cloud_cb_internal(sensor_msgs::msg::PointCloud2::SharedPtr pc_ptr)
{
  std::scoped_lock lock(mutex_);

  // Convert to Open3D Point Cloud
  auto current_pc_ptr = std::make_shared<O3DPointCloud>();
  static constexpr bool kSkipColour = true;
  open3d_conversions::rosToOpen3d(pc_ptr, *current_pc_ptr, kSkipColour);

  // Decimate point cloud and compute normals
  normal_computation_timer_.start();

  const auto & norm_settings = config_.normal_computation;
  if (config_.decimation_factor > 0.0 && config_.decimation_factor < 1.0) {
    current_pc_ptr = current_pc_ptr->RandomDownSample(config_.decimation_factor);
  }
  current_pc_ptr->EstimateNormals(
    open3d::geometry::KDTreeSearchParamHybrid(norm_settings.radius, norm_settings.max_nn),
    norm_settings.fast_normal_computation);

  const auto normal_computation_duration = normal_computation_timer_.stop();
  RCLCPP_INFO_THROTTLE(
    get_logger(), steady_clock_, kLoggingPeriodMs,
    "Current decimation + normal computation time [ms]: %f",
    normal_computation_duration.seconds() * kSecondsToMsScalingFactor);

  // Perform registration in the event that there is a previous point cloud
  if (buffer_pc_ptr_) {
    icp_timer_.start();
    const auto result = perform_registration(*current_pc_ptr, *buffer_pc_ptr_);
    const auto icp_duration = icp_timer_.stop();
    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, kLoggingPeriodMs, "Current registration time [ms]: %f",
      icp_duration.seconds() * kSecondsToMsScalingFactor);

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

}  // namespace r2k_odom_o3d

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_odom_o3d::Open3DOdometryNode)
