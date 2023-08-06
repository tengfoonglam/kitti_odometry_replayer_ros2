#include "ros2_kitti_rviz_plugin/replay_panel.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

namespace r2k_rviz
{

ReplayPanel::ReplayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QGridLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  auto * start_time_label_ptr = new QLabel("Start Time");
  auto * start_time_value_ptr = new QLabel("0.0");

  auto * current_time_label_ptr = new QLabel("Current Time");
  auto * current_time_value_ptr = new QLabel("0.0");

  auto * end_time_label_ptr = new QLabel("End Time");
  auto * end_time_value_ptr = new QLabel("0.0");

  auto * replay_speed_label_ptr = new QLabel("Replay Speed");

  auto * replay_speed_input_ptr = new QDoubleSpinBox();
  replay_speed_input_ptr->setDecimals(2);
  replay_speed_input_ptr->setSingleStep(0.25);
  replay_speed_input_ptr->setRange(0.0, 5.0);
  replay_speed_input_ptr->setValue(1.0);

  auto * step_size_label_ptr = new QLabel("Step Size");

  auto * step_size_input_ptr = new QSpinBox();
  step_size_input_ptr->setSingleStep(1);
  step_size_input_ptr->setMinimum(1);

  auto * play_button_ptr = new QPushButton("Play");
  auto * pause_button_ptr = new QPushButton("Pause");
  auto * step_button_ptr = new QPushButton("Step");

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
}

void ReplayPanel::onInitialize() {}

}  // namespace r2k_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r2k_rviz::ReplayPanel, rviz_common::Panel)
