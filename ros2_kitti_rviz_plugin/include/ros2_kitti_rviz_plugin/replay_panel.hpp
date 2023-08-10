#ifndef ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
#define ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>
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
  static constexpr std::size_t kValueDisplayPrecision{3};
  static constexpr char kValueDisplayFormat{'f'};

  using ReplayerStateMsg = ros2_kitti_msgs::msg::ReplayerState;

  explicit ReplayPanel(QWidget * parent = nullptr);

  ~ReplayPanel();

protected:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Subscription<ReplayerStateMsg>::SharedPtr state_subscriber_ptr_;
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  QLabel * const start_time_label_ptr_;
  QLabel * const start_time_value_ptr_;
  QLabel * const current_time_label_ptr_;
  QLabel * const current_time_value_ptr_;
  QLabel * const end_time_label_ptr_;
  QLabel * const end_time_value_ptr_;
  QLabel * const replay_speed_label_ptr_;
  QDoubleSpinBox * const replay_speed_input_ptr_;
  QLabel * const step_size_label_ptr_;
  QSpinBox * const step_size_input_ptr_;
  QPushButton * const play_button_ptr_;
  QPushButton * const pause_button_ptr_;
  QPushButton * const step_button_ptr_;

  void state_callback(const ReplayerStateMsg::ConstSharedPtr state_ptr);
};

}  // namespace r2k_rviz

#endif  // ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
