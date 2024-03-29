#include "ros2_kitti_replay/kitti_replayer_node.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <filesystem>
#include <image_transport/image_transport.hpp>
#include <ros2_kitti_core/clock_data_loader.hpp>
#include <ros2_kitti_core/image_data_loader.hpp>
#include <ros2_kitti_core/point_cloud_data_loader.hpp>
#include <ros2_kitti_core/pose_data_loader.hpp>
#include <ros2_kitti_msgs/msg/trigger_response.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

namespace r2k_replay
{
const rclcpp::QoS KITTIReplayerNode::kLatchingQoS{
  rclcpp::QoSInitialization{RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 1},
  rmw_qos_profile_t{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, RMW_QOS_DEADLINE_DEFAULT, RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT, false}};

KITTIReplayerNode::KITTIReplayerNode(const rclcpp::NodeOptions & options)
: Node("kitti_replayer", options)
{
  using r2k_core::ClockDataLoader;
  using r2k_core::extract_poses_from_file;
  using r2k_core::extract_timestamps_from_file;
  using r2k_core::ImageDataLoader;
  using r2k_core::PlayDataInterfaceBase;
  using r2k_core::PointCloudDataLoader;
  using r2k_core::PoseDataLoader;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Declare Node params
  declare_parameter("timestamp_path", "");
  declare_parameter("poses_path", "");
  declare_parameter("point_cloud_folder_path", "");
  declare_parameter("gray_image_folder_path", "");
  declare_parameter("colour_image_folder_path", "");
  declare_parameter("ground_truth_data_frame_prefix", kDefaultGroundTruthDataFramePrefix);
  declare_parameter("odometry_data_frame_prefix", kDefaultOdometryDataFramePrefix);
  declare_parameter("publish_point_cloud", true);
  declare_parameter("publish_gray_images", true);
  declare_parameter("publish_colour_images", true);
  declare_parameter("start_time", 0.0);
  declare_parameter("end_time", 0.0);

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
  const auto gray_image_folder_path = std::filesystem::path(
    parameters_client.get_parameter("gray_image_folder_path", std::string{""}));
  const auto colour_image_folder_path = std::filesystem::path(
    parameters_client.get_parameter("colour_image_folder_path", std::string{""}));
  const auto ground_truth_data_frame_prefix = parameters_client.get_parameter(
    "ground_truth_data_frame_prefix", std::string{kDefaultGroundTruthDataFramePrefix});
  const auto odometry_data_frame_prefix = parameters_client.get_parameter(
    "odometry_data_frame_prefix", std::string{kDefaultOdometryDataFramePrefix});
  const bool publish_point_cloud = parameters_client.get_parameter("publish_point_cloud", true);
  const bool publish_gray_images = parameters_client.get_parameter("publish_gray_images", true);
  const bool publish_colour_images = parameters_client.get_parameter("publish_colour_images", true);
  const auto start_time = parameters_client.get_parameter("start_time", 0.0l);
  const auto end_time = parameters_client.get_parameter("end_time", 0.0l);

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
    [pub_ptr = create_publisher<ClockDataLoader::DataType>("clock", kPublisherHistoryDepth)](
      const auto & msg) {
      pub_ptr->publish(msg);
      return true;
    },
    std::move(clock_loader_ptr));
  play_data_interface_check_shutdown_if_fail(*clock_interface_ptr, number_stamps);
  play_data_interface_ptrs.push_back(clock_interface_ptr);

  // Ground Truth Pose (if available)
  ground_truth_path_opt_ = extract_poses_from_file(poses_path);
  if (ground_truth_path_opt_.has_value()) {
    RCLCPP_INFO(get_logger(), "Ground truth path loaded for this dataset");
    PoseDataLoader::Header pose_header;
    pose_header.frame_id = kDefaultGlobalFrame;
    const std::string child_id = add_prefix(ground_truth_data_frame_prefix, "p0");
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
  } else {
    RCLCPP_WARN(get_logger(), "Ground truth path not available for this dataset");
  }

  // Point Cloud
  if (publish_point_cloud) {
    PointCloudDataLoader::Header pc_header;
    pc_header.frame_id = add_prefix(odometry_data_frame_prefix, "lidar");
    auto pc_loader_ptr = std::make_unique<PointCloudDataLoader>(
      "pc_loader", get_logger().get_child("pc_loader"), pc_header);
    pc_loader_ptr->setup(timestamps, point_cloud_folder_path);
    auto pc_interface_ptr = make_shared_interface(
      "pc_interface",
      [pub_ptr = create_publisher<PointCloudDataLoader::DataType>(
         "lidar_pc", kPublisherHistoryDepth)](const auto & msg) {
        pub_ptr->publish(msg);
        return true;
      },
      std::move(pc_loader_ptr));
    play_data_interface_check_shutdown_if_fail(*pc_interface_ptr, number_stamps);
    play_data_interface_ptrs.push_back(pc_interface_ptr);
  }

  // Gray Images
  if (publish_gray_images) {
    auto p0_img_ptr = create_image_play_data_interface(
      add_prefix(odometry_data_frame_prefix, "p0"), "p0_img", gray_image_folder_path / "image_0",
      timestamps);
    play_data_interface_check_shutdown_if_fail(*p0_img_ptr, number_stamps);
    play_data_interface_ptrs.push_back(p0_img_ptr);

    auto p1_img_ptr = create_image_play_data_interface(
      add_prefix(odometry_data_frame_prefix, "p1"), "p1_img", gray_image_folder_path / "image_1",
      timestamps);
    play_data_interface_check_shutdown_if_fail(*p1_img_ptr, number_stamps);
    play_data_interface_ptrs.push_back(p1_img_ptr);
  }

  // Colour Images
  if (publish_colour_images) {
    auto p2_img_ptr = create_image_play_data_interface(
      add_prefix(odometry_data_frame_prefix, "p2"), "p2_img", colour_image_folder_path / "image_2",
      timestamps);
    play_data_interface_check_shutdown_if_fail(*p2_img_ptr, number_stamps);
    play_data_interface_ptrs.push_back(p2_img_ptr);

    auto p3_img_ptr = create_image_play_data_interface(
      add_prefix(odometry_data_frame_prefix, "p3"), "p3_img", colour_image_folder_path / "image_3",
      timestamps);
    play_data_interface_check_shutdown_if_fail(*p3_img_ptr, number_stamps);
    play_data_interface_ptrs.push_back(p3_img_ptr);
  }

  // Create replayer and add interfaces
  const bool specified_time_range = !(start_time == 0.0l && end_time == 0.0l);
  replayer_ptr_ = specified_time_range
                    ? std::make_unique<DataReplayer>(
                        "kitti_replayer", timestamps, get_logger(),
                        DataReplayer::TimeRange(
                          r2k_core::to_timestamp(start_time), r2k_core::to_timestamp(end_time)))
                    : std::make_unique<DataReplayer>("kitti_replayer", timestamps, get_logger());
  for (auto interface_ptr : play_data_interface_ptrs) {
    if (!replayer_ptr_->add_play_data_interface(interface_ptr)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to add %s Play Data Interface to Replayer. Exiting.",
        interface_ptr->name().c_str());
      rclcpp::shutdown();
    }
  }

  // Preparation for publishing transforms
  static_tf_broadcaster_ptr_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  const auto initial_idx = replayer_ptr_->get_replayer_state().start_idx;
  r2k_core::TransformStamped initial_transform_stamped;
  initial_transform_stamped.header.frame_id = std::string{kDefaultGlobalFrame};
  initial_transform_stamped.header.stamp = timestamps.at(initial_idx);
  if (ground_truth_path_opt_.has_value()) {
    initial_transform_stamped.transform = ground_truth_path_opt_.value().at(initial_idx);
  }

  // Publish initial tf of ground truth vehicle w.r.t global frame
  const auto ground_truth_base_link =
    add_prefix(ground_truth_data_frame_prefix, std::string{kVehicleBaseLink});
  auto global_tf_ground_truth_base_link = initial_transform_stamped;
  global_tf_ground_truth_base_link.child_frame_id = ground_truth_base_link;
  static_tf_broadcaster_ptr_->sendTransform(global_tf_ground_truth_base_link);

  // If odometry vehicle available, publish initial pose of odometry vehicle and the odometry
  // frame w.r.t global frame
  const bool has_odometry_vehicle = (ground_truth_data_frame_prefix != odometry_data_frame_prefix);
  if (has_odometry_vehicle) {
    const auto odometry_base_link =
      add_prefix(odometry_data_frame_prefix, std::string{kVehicleBaseLink});
    auto global_tf_odometry_base_link = initial_transform_stamped;
    global_tf_odometry_base_link.child_frame_id = ground_truth_base_link;
    static_tf_broadcaster_ptr_->sendTransform(global_tf_odometry_base_link);

    auto global_tf_odom = initial_transform_stamped;
    global_tf_odom.child_frame_id = std::string{kDefaultOdomFrame};
    static_tf_broadcaster_ptr_->sendTransform(global_tf_odom);
  }

  // Load ground truth pose for visualization (if available)
  if (ground_truth_path_opt_.has_value()) {
    gt_path_pub_ptr_ = create_publisher<nav_msgs::msg::Path>("ground_truth_path", kLatchingQoS);
    const auto state = replayer_ptr_->get_replayer_state();
    const auto & full_path = ground_truth_path_opt_.value();
    const Transforms path_to_visualize{
      std::cbegin(full_path) + state.start_idx, std::cbegin(full_path) + state.end_idx};
    publish_ground_truth_path(path_to_visualize);
  }

  // Add Cb to publish replayer state upon state changes
  auto state_change_cb = [pub_ptr =
                            create_publisher<ReplayerStateMsg>("replayer_state", kLatchingQoS)](
                           const DataReplayer::ReplayerState & replayer_state) {
    pub_ptr->publish(replayer_state_to_msg(replayer_state));
  };
  replayer_ptr_->set_state_change_cb(std::move(state_change_cb));

  // Bind services
  step_service_ptr_ = create_service<StepSrv>(
    "step",
    std::bind(&KITTIReplayerNode::step, this, std::placeholders::_1, std::placeholders::_2));

  play_service_ptr_ = create_service<PlaySrv>(
    "play",
    std::bind(&KITTIReplayerNode::play, this, std::placeholders::_1, std::placeholders::_2));

  pause_service_ptr_ = create_service<TriggerSrv>(
    "pause",
    std::bind(&KITTIReplayerNode::pause, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T>
std::shared_ptr<KITTIReplayerNode::LoadAndPlayDataInterface<T>>
KITTIReplayerNode::make_shared_interface(
  const std::string & name, typename LoadAndPlayDataInterface<T>::PlayCb && cb,
  std::unique_ptr<T> loader_ptr)
{
  return std::make_shared<LoadAndPlayDataInterface<T>>(name, std::move(cb), std::move(loader_ptr));
}

void KITTIReplayerNode::play(
  const std::shared_ptr<PlaySrv::Request> request_ptr,
  std::shared_ptr<PlaySrv::Response> response_ptr)
{
  response_ptr->response.success = replayer_ptr_->play(request_ptr->request.replay_speed);
}

void KITTIReplayerNode::step(
  const std::shared_ptr<StepSrv::Request> request_ptr,
  std::shared_ptr<StepSrv::Response> response_ptr)
{
  response_ptr->response.success = replayer_ptr_->step(DataReplayer::StepRequest(
    request_ptr->request.number_steps, request_ptr->request.replay_speed));
}

void KITTIReplayerNode::pause(
  [[maybe_unused]] const std::shared_ptr<TriggerSrv::Request> request_ptr,
  std::shared_ptr<TriggerSrv::Response> response_ptr)
{
  response_ptr->success = replayer_ptr_->pause();
}

template <typename T>
void KITTIReplayerNode::play_data_interface_check_shutdown_if_fail(
  const LoadAndPlayDataInterface<T> & interface, std::size_t expected_data_size)
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

KITTIReplayerNode::ReplayerStateMsg KITTIReplayerNode::replayer_state_to_msg(
  const DataReplayer::ReplayerState & replayer_state)
{
  ReplayerStateMsg output;
  output.is_playing = replayer_state.playing;
  output.replay_speed = replayer_state.replay_speed;
  output.start_time = replayer_state.start_time;
  output.current_time = replayer_state.current_time;
  output.target_time = replayer_state.target_time;
  output.end_time = replayer_state.end_time;
  output.start_idx = replayer_state.start_idx;
  output.next_idx = replayer_state.next_idx;
  output.target_idx = replayer_state.target_idx;
  output.end_idx = replayer_state.end_idx;
  return output;
}

void KITTIReplayerNode::publish_ground_truth_path(const Transforms & transforms)
{
  if (!gt_path_pub_ptr_) {
    RCLCPP_WARN(
      get_logger(), "Ground truth path publisher no initialized yet, cannot publish path.");
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = std::string{kDefaultGlobalFrame};

  auto msg_ptr = std::make_unique<nav_msgs::msg::Path>();
  msg_ptr->header = header;
  msg_ptr->poses.resize(transforms.size());

  using PoseStamped = decltype(msg_ptr->poses)::value_type;
  std::transform(
    std::cbegin(transforms), std::cend(transforms), std::begin(msg_ptr->poses),
    [&header](const auto & transform) {
      PoseStamped pose_stamped;
      pose_stamped.header = header;
      pose_stamped.pose.position.x = transform.translation.x;
      pose_stamped.pose.position.y = transform.translation.y;
      pose_stamped.pose.position.z = transform.translation.z;
      pose_stamped.pose.orientation.w = transform.rotation.w;
      pose_stamped.pose.orientation.x = transform.rotation.x;
      pose_stamped.pose.orientation.y = transform.rotation.y;
      pose_stamped.pose.orientation.z = transform.rotation.z;
      return pose_stamped;
    });

  gt_path_pub_ptr_->publish(std::move(msg_ptr));
}

std::shared_ptr<KITTIReplayerNode::ImageLoadAndPlayInterface>
KITTIReplayerNode::create_image_play_data_interface(
  const std::string & frame_id, const std::string & topic_name,
  const std::filesystem::path & folder_path, const r2k_core::Timestamps & timestamps)
{
  using r2k_core::ImageDataLoader;
  ImageDataLoader::Header header;
  header.frame_id = frame_id;
  auto img_loader_ptr = std::make_unique<ImageDataLoader>(
    topic_name + "_loader", get_logger().get_child(topic_name), header);
  img_loader_ptr->setup(timestamps, folder_path);
  auto interface_ptr = make_shared_interface(
    topic_name + "_interface",
    [pub = image_transport::create_publisher(this, topic_name)](const auto & msg) {
      pub.publish(msg);
      return true;
    },
    std::move(img_loader_ptr));
  return interface_ptr;
}

std::string KITTIReplayerNode::add_prefix(const std::string & prefix, const std::string & segment)
{
  return prefix.empty() ? segment : (prefix + "/" + segment);
}

}  // namespace r2k_replay

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(r2k_replay::KITTIReplayerNode)
