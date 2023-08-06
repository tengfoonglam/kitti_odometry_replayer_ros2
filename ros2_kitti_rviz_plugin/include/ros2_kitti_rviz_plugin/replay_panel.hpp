#ifndef ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
#define ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_

#include <QMainWindow>
#include <rviz_common/panel.hpp>

namespace r2k_rviz
{

class ReplayPanel : public rviz_common::Panel  // QMainWindow
{
  Q_OBJECT

public:
  explicit ReplayPanel(QWidget * parent = nullptr);

  void onInitialize() final;
};

}  // namespace r2k_rviz

#endif  // ROS2_KITTI_RVIZ_PLUGIN__REPLAY_PANEL_HPP_
