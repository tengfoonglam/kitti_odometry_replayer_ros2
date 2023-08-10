#ifndef ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
#define ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_

#include <QMainWindow>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_msgs/msg/replayer_state.hpp>
#include <rviz_common/panel.hpp>
#include <thread>

namespace r2k_rviz
{

class ReplayPanel : public rviz_common::Panel  // QMainWindow
{
  Q_OBJECT

public:
  static constexpr const char kReplayerStateTopicName[]{"replayer_state"};

  using ReplayerStateMsg = ros2_kitti_msgs::msg::ReplayerState;

  explicit ReplayPanel(QWidget * parent = nullptr);

  void onInitialize() final;

  ~ReplayPanel();

protected:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<ReplayerStateMsg>::SharedPtr state_subscriber_ptr_;
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  void state_callback(const ReplayerStateMsg::ConstSharedPtr state);
};

}  // namespace r2k_rviz

#endif  // ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
