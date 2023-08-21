#ifndef ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_
#define ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

namespace r2k_viz_tools
{

class PathPublisher
{
public:
  using Marker = visualization_msgs::msg::Marker;

  PathPublisher(rclcpp::Node::SharedPtr node_ptr, const std::string & topic_name);

  void publish(const geometry_msgs::msg::TransformStamped & transform_stamped);

  void reset();

private:
  std::mutex mutex_;
  std::size_t line_id_{0};
  std::size_t pose_id_{0};
  std::optional<geometry_msgs::msg::TransformStamped> previous_transform_stamped_;
  rclcpp::Publisher<Marker>::SharedPtr publisher_ptr_;

  void publish_pose(const geometry_msgs::msg::TransformStamped & transform_stamped);

  void publish_line(const geometry_msgs::msg::TransformStamped & transform_stamped);
};

}  // namespace r2k_viz_tools

#endif  // ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_
