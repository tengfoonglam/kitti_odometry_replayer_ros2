#include <rclcpp/rclcpp.hpp>
#include <ros2_kitti_replay/kitti_replayer_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<r2k_replay::KITTIReplayerNode>(options));
  rclcpp::shutdown();
  return 0;
}
