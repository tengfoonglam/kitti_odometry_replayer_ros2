#include "ros2_kitti_rviz_plugin/replay_panel.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QVBoxLayout>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>
#include <rviz_common/visualization_manager.hpp>

namespace r2k_rviz
{

ReplayPanel::ReplayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  auto text = new QLabel("This is a demo");
  auto lineEdit = new QLineEdit();
  auto radio = new QRadioButton();

  layout->addWidget(text);
  layout->addWidget(lineEdit);
  layout->addWidget(radio);

  setLayout(layout);
}

void ReplayPanel::onInitialize() {}

}  // namespace r2k_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r2k_rviz::ReplayPanel, rviz_common::Panel)
