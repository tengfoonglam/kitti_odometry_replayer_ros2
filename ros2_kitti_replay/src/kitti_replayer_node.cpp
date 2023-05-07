#include "ros2_kitti_replay/kitti_replayer_node.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <filesystem>
#include <ros2_kitti_interface/msg/trigger_response.hpp>
#include <vector>

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
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the parameters service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Parameters service not available, waiting again...");
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
      get_logger(), "Failed to load timestamps from \"%s\". Exiting.",
      timestamp_path.string().c_str());
    rclcpp::shutdown();
  }
  const auto & timestamps = timestamps_opt.value();
  const auto number_stamps = timestamps.size();

  // Create load and play interfaces
  std::vector<std::shared_ptr<PlayDataInterfaceBase>> play_data_interface_ptrs;

  // Clock
  auto clock_loader_ptr =
    std::make_unique<ClockDataLoader>("clock_loader", get_logger().get_child("clock_loader"));
  clock_loader_ptr->setup(timestamps, "");
  auto clock_interface_ptr = make_shared_interface(
    "clock_interface",
    [pub_ptr = this->create_publisher<ClockDataLoader::DataType>("clock", 10)](const auto & msg) {
      pub_ptr->publish(msg);
      return true;
    },
    std::move(clock_loader_ptr));
  play_data_interface_check_shutdown_if_fail(*clock_interface_ptr, number_stamps);
  play_data_interface_ptrs.push_back(clock_interface_ptr);

  // Ground Truth Pose
  r2k_replay::PoseDataLoader::Header pose_header;
  pose_header.frame_id = "map";
  const std::string child_id{"p0"};
  auto pose_loader_ptr = std::make_unique<PoseDataLoader>(
    "pose_loader", get_logger().get_child("pose_loader"), pose_header, child_id);
  pose_loader_ptr->setup(timestamps, poses_path);
  auto pose_interface_ptr = make_shared_interface(
    "pose_interface",
    [broadcast = tf2_ros::TransformBroadcaster(*this)](const auto & msg) mutable {
      broadcast.sendTransform(msg);
      return true;
    },
    std::move(pose_loader_ptr));
  play_data_interface_check_shutdown_if_fail(*pose_interface_ptr, number_stamps);
  play_data_interface_ptrs.push_back(pose_interface_ptr);

  // Point Cloud
  r2k_replay::PointCloudDataLoader::Header pc_header;
  pc_header.frame_id = "lidar";
  auto pc_loader_ptr = std::make_unique<PointCloudDataLoader>(
    "pc_loader", get_logger().get_child("pc_loader"), pc_header);
  pc_loader_ptr->setup(timestamps, point_cloud_folder_path);
  auto pc_interface_ptr = make_shared_interface(
    "pc_interface",
    [pub_ptr =
       this->create_publisher<PointCloudDataLoader::DataType>("lidar_pc", 10)](const auto & msg) {
      pub_ptr->publish(msg);
      return true;
    },
    std::move(pc_loader_ptr));
  play_data_interface_check_shutdown_if_fail(*pc_interface_ptr, number_stamps);
  play_data_interface_ptrs.push_back(pc_interface_ptr);

  // Create replayer and add interfaces
  replayer_ptr_ = std::make_unique<DataReplayer>("kitti_replayer", timestamps);
  for (auto interface_ptr : play_data_interface_ptrs) {
    if (!replayer_ptr_->add_play_data_interface(interface_ptr)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to add %s Play Data Interface to Replayer. Exiting.",
        interface_ptr->name().c_str());
      rclcpp::shutdown();
    }
  }

  // Add Cb to publish replayer state upon state changes
  auto state_change_cb = [pub_ptr = this->create_publisher<ReplayerStateMsg>("replayer_state", 10)](
                           const DataReplayer::ReplayerState & replayer_state) {
    pub_ptr->publish(replayer_state_to_msg(replayer_state));
  };
  replayer_ptr_->set_state_change_cb(std::move(state_change_cb));

  // Bind services
  resume_service_ptr = this->create_service<ros2_kitti_interface::srv::Resume>(
    "~/resume",
    std::bind(&KITTIReplayerNode::resume, this, std::placeholders::_1, std::placeholders::_2));

  step_service_ptr = this->create_service<ros2_kitti_interface::srv::Step>(
    "~/step",
    std::bind(&KITTIReplayerNode::step, this, std::placeholders::_1, std::placeholders::_2));

  pause_service_ptr = this->create_service<std_srvs::srv::Trigger>(
    "~/pause",
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

void KITTIReplayerNode::step(
  const std::shared_ptr<ros2_kitti_interface::srv::Step::Request> request_ptr,
  std::shared_ptr<ros2_kitti_interface::srv::Step::Response> response_ptr)
{
  response_ptr->response.success = replayer_ptr_->step(DataReplayer::StepRequest(
    request_ptr->request.number_steps, request_ptr->request.replay_speed));
}

void KITTIReplayerNode::pause(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request_ptr,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response_ptr)
{
  response_ptr->success = replayer_ptr_->pause();
}

template <typename T>
void KITTIReplayerNode::play_data_interface_check_shutdown_if_fail(
  const LoadAndPlayDataInterface<T> & interface, const std::size_t expected_data_size)
{
  const auto interface_name = interface.name();

  if (expected_data_size != interface.data_size()) {
    RCLCPP_ERROR(
      get_logger(),
      "%s Play Data Interface loaded a number of readings (%zu) that is different from the "
      "number of timestamps (%zu). Exiting.",
      interface_name.c_str(), expected_data_size, interface.data_size());
    rclcpp::shutdown();
  }

  if (!interface.ready()) {
    RCLCPP_ERROR(
      get_logger(), "%s Play Data Interface failed to setup. Exiting.", interface_name.c_str());
    rclcpp::shutdown();
  }
}

[[nodiscard]] KITTIReplayerNode::ReplayerStateMsg KITTIReplayerNode::replayer_state_to_msg(
  const DataReplayer::ReplayerState & replayer_state)
{
  ReplayerStateMsg output;
  output.is_playing = replayer_state.playing;
  output.replay_speed = replayer_state.replay_speed;
  output.start_time = replayer_state.start_time;
  output.current_time = replayer_state.current_time;
  output.final_time = replayer_state.final_time;
  output.next_idx = replayer_state.next_idx;
  output.target_idx = replayer_state.target_idx;
  output.data_size = replayer_state.data_size;
  return output;
}

}  // namespace r2k_replay
