#ifndef ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_
#define ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_interface/srv/resume.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "ros2_kitti_replay/data_replayer.hpp"
#include "ros2_kitti_replay/load_and_play_data_interface.hpp"

namespace r2k_replay
{

class KITTIReplayerNode : public rclcpp::Node
{
public:
  explicit KITTIReplayerNode(const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<DataReplayer> replayer_ptr_;
  rclcpp::Service<ros2_kitti_interface::srv::Resume>::SharedPtr resume_service_ptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_ptr;

  template <typename T>
  static std::shared_ptr<LoadAndPlayDataInterface<T>> make_shared_interface(
    const std::string & name, typename LoadAndPlayDataInterface<T>::PlayCb && cb,
    std::unique_ptr<T> loader_ptr);

  void resume(
    const std::shared_ptr<ros2_kitti_interface::srv::Resume::Request> request_ptr,
    std::shared_ptr<ros2_kitti_interface::srv::Resume::Response> response_ptr);

  void pause(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request_ptr,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response_ptr);
};

}  // namespace r2k_replay

#endif  // ROS2_KITTI_REPLAY__KITTI_REPLAYER_NODE_HPP_
