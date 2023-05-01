#include "ros2_kitti_replay/kitti_replayer_node.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <filesystem>
#include <ros2_kitti_interface/msg/trigger_response.hpp>

#include "ros2_kitti_replay/clock_data_loader.hpp"
#include "ros2_kitti_replay/point_cloud_data_loader.hpp"
#include "ros2_kitti_replay/pose_data_loader.hpp"
#include "ros2_kitti_replay/timestamp_utils.hpp"

namespace r2k_replay
{

KITTIReplayerNode::KITTIReplayerNode(const rclcpp::NodeOptions & options)
: Node("kitti_replayer", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  this->declare_parameter("timestamp_path", "");
  this->declare_parameter("poses_path", "");
  this->declare_parameter("point_cloud_folder_path", "");

  // Wait for parameters to be loaded
  auto parameters_client = rclcpp::SyncParametersClient(this);
  while (!parameters_client.wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(), "Interrupted while waiting for the parameters service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Parameters service not available, waiting again...");
  }

  // Get params
  const auto timestamp_path =
    std::filesystem::path(parameters_client.get_parameter("timestamp_path", std::string{""}));
  const auto poses_path =
    std::filesystem::path(parameters_client.get_parameter("poses_path", std::string{""}));
  const auto point_cloud_folder_path = std::filesystem::path(
    parameters_client.get_parameter("point_cloud_folder_path", std::string{""}));

  // Load timestamps
  auto timestamps_opt = extract_timestamps_from_file(timestamp_path);
  if (!timestamps_opt.has_value()) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load timestamps from \"%s\". Exiting.",
      timestamp_path.string().c_str());
    rclcpp::shutdown();
  }
  const auto & timestamps = timestamps_opt.value();

  // Create data loaders
  auto clock_loader_ptr =
    std::make_unique<ClockDataLoader>("clock_loader", this->get_logger().get_child("clock_loader"));

  r2k_replay::PoseDataLoader::Header pose_header;
  pose_header.frame_id = "map";
  const std::string child_id{"p0"};
  auto pose_loader_ptr = std::make_unique<PoseDataLoader>(
    "pose_loader", this->get_logger().get_child("pose_loader"), pose_header, child_id);

  r2k_replay::PointCloudDataLoader::Header pc_header;
  pc_header.frame_id = "lidar";
  auto pc_loader_ptr = std::make_unique<PointCloudDataLoader>(
    "pc_loader", this->get_logger().get_child("pc_loader"), pc_header);

  const auto clock_loader_setup_success = clock_loader_ptr->setup(timestamps, "");
  const auto pose_loader_setup_success = pose_loader_ptr->setup(timestamps, poses_path);
  const auto pc_loader_setup_success = pc_loader_ptr->setup(timestamps, point_cloud_folder_path);

  // Check if loaders have been setup correctly
  if (!clock_loader_setup_success || !pose_loader_setup_success || !pc_loader_setup_success) {
    RCLCPP_ERROR(this->get_logger(), "One of the loaders failed to setup. Exiting.");
    rclcpp::shutdown();
  }

  const auto number_stamps = timestamps.size();
  if (
    number_stamps != clock_loader_ptr->data_size() ||
    number_stamps != pose_loader_ptr->data_size() || number_stamps != pc_loader_ptr->data_size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "One of the loaders loaded a number of readings that are different than then number of "
      "timestamps. Exiting.");
    rclcpp::shutdown();
  }

  // Create load and play interfaces
  auto clock_interface_ptr = make_shared_interface(
    "clock_interface",
    [pub_ptr = this->create_publisher<ClockDataLoader::DataType>("clock", 10)](const auto & msg) {
      pub_ptr->publish(msg);
      return true;
    },
    std::move(clock_loader_ptr));

  auto pose_interface_ptr = make_shared_interface(
    "pose_interface",
    [broadcast = tf2_ros::TransformBroadcaster(*this)](const auto & msg) mutable {
      broadcast.sendTransform(msg);
      return true;
    },
    std::move(pose_loader_ptr));

  auto pc_interface_ptr = make_shared_interface(
    "pc_interface",
    [pub_ptr =
       this->create_publisher<PointCloudDataLoader::DataType>("lidar_pc", 10)](const auto & msg) {
      pub_ptr->publish(msg);
      return true;
    },
    std::move(pc_loader_ptr)

  );

  // Create replayer
  replayer_ptr_ = std::make_unique<DataReplayer>("kitti_replayer", timestamps);
  // TODO(TF): Check if these are added using return bool
  replayer_ptr_->add_play_data_interface(clock_interface_ptr);
  replayer_ptr_->add_play_data_interface(pose_interface_ptr);
  replayer_ptr_->add_play_data_interface(pc_interface_ptr);

  // Add Cb to publish replayer state upon state changes

  // Bind services
  resume_service_ptr = this->create_service<ros2_kitti_interface::srv::Resume>(
    "resume",
    std::bind(&KITTIReplayerNode::resume, this, std::placeholders::_1, std::placeholders::_2));

  pause_service_ptr = this->create_service<std_srvs::srv::Trigger>(
    "pause",
    std::bind(&KITTIReplayerNode::pause, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T>
std::shared_ptr<LoadAndPlayDataInterface<T>> KITTIReplayerNode::make_shared_interface(
  const std::string & name, typename LoadAndPlayDataInterface<T>::PlayCb && cb,
  std::unique_ptr<T> loader_ptr)
{
  return std::make_shared<LoadAndPlayDataInterface<T>>(name, std::move(cb), std::move(loader_ptr));
}

void KITTIReplayerNode::resume(
  const std::shared_ptr<ros2_kitti_interface::srv::Resume::Request> request_ptr,
  std::shared_ptr<ros2_kitti_interface::srv::Resume::Response> response_ptr)
{
  response_ptr->response.success = replayer_ptr_->resume(request_ptr->request.replay_speed);
}

void KITTIReplayerNode::pause(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request_ptr,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response_ptr)
{
  response_ptr->success = replayer_ptr_->pause();
}

}  // namespace r2k_replay
