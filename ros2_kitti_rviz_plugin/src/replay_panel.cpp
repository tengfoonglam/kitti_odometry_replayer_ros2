#include "ros2_kitti_rviz_plugin/replay_panel.hpp"

#include <QString>
#include <functional>
#include <rclcpp/time.hpp>

namespace r2k_rviz
{

const rclcpp::QoS ReplayPanel::kLatchingQoS{
  rclcpp::QoSInitialization{RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 1},
  rmw_qos_profile_t{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, RMW_QOS_DEADLINE_DEFAULT, RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT, false}};

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
  setup_ros();
  setup_widgets();
  setup_layout();
}

void ReplayPanel::setup_ros()
{
  node_ptr_ = std::make_shared<rclcpp::Node>("replay_panel_node");

  state_subscriber_ptr_ =
    node_ptr_->create_subscription<decltype(state_subscriber_ptr_)::element_type::SubscribedType>(
      std::string{kReplayerStateTopicName}, kLatchingQoS,
      std::bind(&ReplayPanel::state_callback, this, std::placeholders::_1));

  play_client_ptr_ = node_ptr_->create_client<PlaySrv>("play");
  pause_client_ptr_ = node_ptr_->create_client<TriggerSrv>("pause");
  step_client_ptr_ = node_ptr_->create_client<StepSrv>("step");

  executor_.add_node(node_ptr_);
  executor_thread_ = std::thread([this]() { executor_.spin(); });
}

void ReplayPanel::setup_widgets()
{
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

  QObject::connect(play_button_ptr_, &QPushButton::clicked, this, &ReplayPanel::on_play_clicked);
  QObject::connect(pause_button_ptr_, &QPushButton::clicked, this, &ReplayPanel::on_pause_clicked);
  QObject::connect(step_button_ptr_, &QPushButton::clicked, this, &ReplayPanel::on_step_clicked);
}

void ReplayPanel::on_play_clicked()
{
  auto request_ptr = std::make_shared<PlaySrv::Request>();
  request_ptr->request.replay_speed = replay_speed_input_ptr_->value();
  play_client_ptr_->async_send_request(request_ptr);
}

void ReplayPanel::on_pause_clicked()
{
  pause_client_ptr_->async_send_request(std::make_shared<TriggerSrv::Request>());
}

void ReplayPanel::on_step_clicked()
{
  auto request_ptr = std::make_shared<StepSrv::Request>();
  request_ptr->request.replay_speed = replay_speed_input_ptr_->value();
  request_ptr->request.number_steps =
    static_cast<decltype(request_ptr->request.number_steps)>(step_size_input_ptr_->value());
  step_client_ptr_->async_send_request(request_ptr);
}

void ReplayPanel::setup_layout()
{
  auto * const layout = new QGridLayout();
  layout->setContentsMargins(0, 0, 0, 0);
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
}

inline void set_timestamp_as_label(QLabel & label, const double & seconds)
{
  label.setText(QString::number(
    seconds, ReplayPanel::kValueDisplayFormat, ReplayPanel::kValueDisplayPrecision));
}

void ReplayPanel::state_callback(const ReplayerStateMsg::ConstSharedPtr state_ptr)
{
  const auto start_time = rclcpp::Time(state_ptr->start_time).seconds();
  const auto current_time = rclcpp::Time(state_ptr->current_time).seconds();
  const auto end_time = rclcpp::Time(state_ptr->end_time).seconds();

  set_timestamp_as_label(*start_time_value_ptr_, start_time);
  set_timestamp_as_label(*current_time_value_ptr_, current_time);
  set_timestamp_as_label(*end_time_value_ptr_, end_time);

  if (current_time >= end_time) {
    play_button_ptr_->setDisabled(true);
    pause_button_ptr_->setDisabled(true);
    step_button_ptr_->setDisabled(true);
  } else if (state_ptr->is_playing) {
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
