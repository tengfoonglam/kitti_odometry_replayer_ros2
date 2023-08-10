#include "ros2_kitti_rviz_plugin/replay_panel.hpp"

#include <QString>
#include <functional>
#include <rclcpp/time.hpp>

namespace r2k_rviz
{

ReplayPanel::ReplayPanel(QWidget * parent)
: rviz_common::Panel(parent),
  start_time_label_ptr_(new QLabel("Start Time [s]")),
  start_time_value_ptr_(new QLabel("0.0")),
  current_time_label_ptr_(new QLabel("Current Time [s]")),
  current_time_value_ptr_(new QLabel("0.0")),
  end_time_label_ptr_(new QLabel("End Time [s]")),
  end_time_value_ptr_(new QLabel("0.0")),
  replay_speed_label_ptr_(new QLabel("Replay Speed")),
  replay_speed_input_ptr_(new QDoubleSpinBox()),
  step_size_label_ptr_(new QLabel("Step Size")),
  step_size_input_ptr_(new QSpinBox()),
  play_button_ptr_(new QPushButton("Play")),
  pause_button_ptr_(new QPushButton("Pause")),
  step_button_ptr_(new QPushButton("Step"))
{
  auto * const layout = new QGridLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  replay_speed_input_ptr_->setDecimals(2);
  replay_speed_input_ptr_->setSingleStep(0.25);
  replay_speed_input_ptr_->setRange(0.25, 5.0);
  replay_speed_input_ptr_->setValue(1.0);
  replay_speed_input_ptr_->setPrefix("x");
  step_size_input_ptr_->setSingleStep(1);
  step_size_input_ptr_->setMinimum(1);
  play_button_ptr_->setDisabled(true);
  pause_button_ptr_->setDisabled(true);
  step_button_ptr_->setDisabled(true);

  layout->addWidget(start_time_label_ptr_, 0, 0);
  layout->addWidget(start_time_value_ptr_, 0, 1);
  layout->addWidget(current_time_label_ptr_, 1, 0);
  layout->addWidget(current_time_value_ptr_, 1, 1);
  layout->addWidget(end_time_label_ptr_, 2, 0);
  layout->addWidget(end_time_value_ptr_, 2, 1);
  layout->addWidget(replay_speed_label_ptr_, 3, 0);
  layout->addWidget(replay_speed_input_ptr_, 3, 1);
  layout->addWidget(step_size_label_ptr_, 4, 0);
  layout->addWidget(step_size_input_ptr_, 4, 1);
  layout->addWidget(play_button_ptr_, 5, 0);
  layout->addWidget(step_button_ptr_, 5, 1);
  layout->addWidget(pause_button_ptr_, 6, 0, 1, 2);

  setLayout(layout);

  node_ptr_ = std::make_shared<rclcpp::Node>("replay_panel_node");

  state_subscriber_ptr_ = node_ptr_->create_subscription<ReplayerStateMsg>(
    std::string{kReplayerStateTopicName}, 10,
    std::bind(&ReplayPanel::state_callback, this, std::placeholders::_1));

  executor_.add_node(node_ptr_);
  executor_thread_ = std::thread([this]() { executor_.spin(); });
}

template <typename T>
inline void set_timestamp_as_text(QLabel & label, const T & timestamp_msg)
{
  label.setText(QString::number(
    rclcpp::Time(timestamp_msg).seconds(), ReplayPanel::kValueDisplayFormat,
    ReplayPanel::kValueDisplayPrecision));
}

void ReplayPanel::state_callback(const ReplayerStateMsg::ConstSharedPtr state_ptr)
{
  set_timestamp_as_text(*start_time_value_ptr_, state_ptr->start_time);
  set_timestamp_as_text(*current_time_value_ptr_, state_ptr->current_time);
  set_timestamp_as_text(*end_time_value_ptr_, state_ptr->end_time);

  if (state_ptr->is_playing) {
    play_button_ptr_->setDisabled(true);
    pause_button_ptr_->setDisabled(false);
    step_button_ptr_->setDisabled(true);
  } else {
    play_button_ptr_->setDisabled(false);
    pause_button_ptr_->setDisabled(true);
    step_button_ptr_->setDisabled(false);
  }
}

ReplayPanel::~ReplayPanel()
{
  executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

}  // namespace r2k_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r2k_rviz::ReplayPanel, rviz_common::Panel)
