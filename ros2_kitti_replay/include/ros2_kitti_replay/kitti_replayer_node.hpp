#ifndef ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_
#define ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_interface/msg/replayer_state.hpp>
#include <ros2_kitti_interface/srv/play.hpp>
#include <ros2_kitti_interface/srv/set_time_range.hpp>
#include <ros2_kitti_interface/srv/step.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "ros2_kitti_replay/data_replayer.hpp"
#include "ros2_kitti_replay/load_and_play_data_interface.hpp"

namespace r2k_replay
{

class KITTIReplayerNode : public rclcpp::Node
{
public:
  using ReplayerStateMsg = ros2_kitti_interface::msg::ReplayerState;

  explicit KITTIReplayerNode(const rclcpp::NodeOptions & options);

  [[nodiscard]] static ReplayerStateMsg replayer_state_to_msg(
    const DataReplayer::ReplayerState & replayer_state);

private:
  std::unique_ptr<DataReplayer> replayer_ptr_;
  std::shared_ptr<rclcpp::Publisher<ReplayerStateMsg>> state_publisher_ptr_;
  rclcpp::Service<ros2_kitti_interface::srv::Play>::SharedPtr play_service_ptr;
  rclcpp::Service<ros2_kitti_interface::srv::Step>::SharedPtr step_service_ptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_ptr;
  rclcpp::Service<ros2_kitti_interface::srv::SetTimeRange>::SharedPtr set_time_range_service_ptr;

  template <typename T>
  static std::shared_ptr<LoadAndPlayDataInterface<T>> make_shared_interface(
    const std::string & name, typename LoadAndPlayDataInterface<T>::PlayCb && cb,
    std::unique_ptr<T> loader_ptr);

  template <typename T>
  void play_data_interface_check_shutdown_if_fail(
    const LoadAndPlayDataInterface<T> & interface, const std::size_t expected_data_size);

  void play(
    const std::shared_ptr<ros2_kitti_interface::srv::Play::Request> request_ptr,
    std::shared_ptr<ros2_kitti_interface::srv::Play::Response> response_ptr);

  void step(
    const std::shared_ptr<ros2_kitti_interface::srv::Step::Request> request_ptr,
    std::shared_ptr<ros2_kitti_interface::srv::Step::Response> response_ptr);

  void pause(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request_ptr,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response_ptr);

  void set_time_range(
    const std::shared_ptr<ros2_kitti_interface::srv::SetTimeRange::Request> request_ptr,
    std::shared_ptr<ros2_kitti_interface::srv::SetTimeRange::Response> response_ptr);
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_
