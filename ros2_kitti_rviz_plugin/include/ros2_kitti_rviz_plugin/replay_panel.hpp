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
#include <ros2_kitti_msgs/srv/play.hpp>
#include <ros2_kitti_msgs/srv/step.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

namespace r2k_rviz
{

class ReplayPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  static constexpr const char kReplayerStateTopicName[]{"replayer_state"};
  static constexpr std::size_t kValueDisplayPrecision{3};
  static constexpr char kValueDisplayFormat{'f'};
  static const rclcpp::QoS kLatchingQoS;

  using ReplayerStateMsg = ros2_kitti_msgs::msg::ReplayerState;
  using PlaySrv = ros2_kitti_msgs::srv::Play;
  using StepSrv = ros2_kitti_msgs::srv::Step;
  using TriggerSrv = std_srvs::srv::Trigger;

  /**
   * @brief Construct a new Replay Panel object
   *
   * @param parent
   */
  explicit ReplayPanel(QWidget * parent = nullptr);

  ~ReplayPanel();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Subscription<ReplayerStateMsg>::SharedPtr state_subscriber_ptr_;
  std::thread executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<PlaySrv>::SharedPtr play_client_ptr_;
  rclcpp::Client<TriggerSrv>::SharedPtr pause_client_ptr_;
  rclcpp::Client<StepSrv>::SharedPtr step_client_ptr_;

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

  // Setup methods
  void setup_ros();
  void setup_widgets();
  void setup_layout();

  // Click callbacks
  void on_play_clicked();
  void on_pause_clicked();
  void on_step_clicked();

  /**
   * @brief Callback when there is a replayer state change
   *
   * @param state_ptr
   */
  void state_callback(const ReplayerStateMsg::ConstSharedPtr state_ptr);
};

}  // namespace r2k_rviz

#endif  // ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
