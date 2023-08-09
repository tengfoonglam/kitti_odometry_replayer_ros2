#include "ros2_kitti_rviz_plugin/replay_panel.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <functional>

namespace r2k_rviz
{

ReplayPanel::ReplayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * const layout = new QGridLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  auto * const start_time_label_ptr = new QLabel("Start Time");
  auto * const start_time_value_ptr = new QLabel("0.0");

  auto * const current_time_label_ptr = new QLabel("Current Time");
  auto * const current_time_value_ptr = new QLabel("0.0");

  auto * const end_time_label_ptr = new QLabel("End Time");
  auto * const end_time_value_ptr = new QLabel("0.0");

  auto * const replay_speed_label_ptr = new QLabel("Replay Speed");

  auto * const replay_speed_input_ptr = new QDoubleSpinBox();
  replay_speed_input_ptr->setDecimals(2);
  replay_speed_input_ptr->setSingleStep(0.25);
  replay_speed_input_ptr->setRange(0.0, 5.0);
  replay_speed_input_ptr->setValue(1.0);

  auto * const step_size_label_ptr = new QLabel("Step Size");

  auto * const step_size_input_ptr = new QSpinBox();
  step_size_input_ptr->setSingleStep(1);
  step_size_input_ptr->setMinimum(1);

  auto * const play_button_ptr = new QPushButton("Play");
  play_button_ptr->setDisabled(true);

  auto * const pause_button_ptr = new QPushButton("Pause");
  pause_button_ptr->setDisabled(true);

  auto * const step_button_ptr = new QPushButton("Step");
  step_button_ptr->setDisabled(true);

  layout->addWidget(start_time_label_ptr, 0, 0);
  layout->addWidget(start_time_value_ptr, 0, 1);

  layout->addWidget(current_time_label_ptr, 1, 0);
  layout->addWidget(current_time_value_ptr, 1, 1);

  layout->addWidget(end_time_label_ptr, 2, 0);
  layout->addWidget(end_time_value_ptr, 2, 1);

  layout->addWidget(replay_speed_label_ptr, 3, 0);
  layout->addWidget(replay_speed_input_ptr, 3, 1);

  layout->addWidget(step_size_label_ptr, 4, 0);
  layout->addWidget(step_size_input_ptr, 4, 1);

  layout->addWidget(play_button_ptr, 5, 0);
  layout->addWidget(step_button_ptr, 5, 1);

  layout->addWidget(pause_button_ptr, 6, 0, 1, 2);

  setLayout(layout);

  node_ptr_ = std::make_shared<rclcpp::Node>("replay_panel_node");

  state_subscriber_ptr_ = node_ptr_->create_subscription<ReplayerStateMsg>(
    std::string{kReplayerStateTopicName}, 10,
    std::bind(&ReplayPanel::state_callback, this, std::placeholders::_1));

  spin_thread_ = std::thread([this]() {
    while (!shutdown_flag_) {
      rclcpp::spin_some(node_ptr_);
    }
    // RCLCPP_INFO(node_ptr_->get_logger(), "Replay Panel Spin Thread Completed");
  });
}

void ReplayPanel::state_callback(const ReplayerStateMsg::ConstSharedPtr state)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "HOI");
}

void ReplayPanel::onInitialize() {}

ReplayPanel::~ReplayPanel()
{
  shutdown_flag_ = true;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  rclcpp::shutdown();
}

}  // namespace r2k_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r2k_rviz::ReplayPanel, rviz_common::Panel)
