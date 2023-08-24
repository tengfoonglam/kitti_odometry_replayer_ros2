#ifndef ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_
#define ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <string>
#include <tuple>

namespace r2k_viz_tools
{

class PathPublisher
{
public:
  using Point = geometry_msgs::msg::Point;
  using Pose = geometry_msgs::msg::Pose;
  using Transform = geometry_msgs::msg::Transform;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using Vector3 = geometry_msgs::msg::Vector3;

  static constexpr double kAxisLength{0.25};
  static constexpr double kAxisRadius{0.25};
  static constexpr double kPathRadius{0.22};
  static constexpr rviz_visual_tools::Colors kPathColour{rviz_visual_tools::Colors::WHITE};

  PathPublisher(
    rclcpp::Node * const node_ptr, const std::string & base_frame_id,
    const std::string & topic_name);

  void publish(const TransformStamped & transform_stamped);

  void reset();

private:
  std::mutex mutex_;
  std::optional<Point> previous_point_opt_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_ptr_;

  [[nodiscard]] static Pose transform_to_pose(const Transform & transform);

  [[nodiscard]] static Point vector3_to_point(const Vector3 & vector_3);
};

}  // namespace r2k_viz_tools

#endif  // ROS2_KITTI_VISUALIZATION_TOOLS__PATH_PUBLISHER_HPP_
