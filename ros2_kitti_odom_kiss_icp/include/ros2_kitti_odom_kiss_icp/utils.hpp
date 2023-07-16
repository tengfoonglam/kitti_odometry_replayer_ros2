// Adapted from KISS ICP Repository https:// github.com/PRBonn/kiss-icp

// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ROS2_KITTI_ODOM_KISS_ICP__UTILS_HPP_
#define ROS2_KITTI_ODOM_KISS_ICP__UTILS_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <kiss_icp/pipeline/KissICP.hpp>
#include <regex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>

namespace r2k_odom_kiss_icp
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

[[nodiscard]] inline std::string fix_frame_id(const std::string & frame_id)
{
  return std::regex_replace(frame_id, std::regex("^/"), "");
}

[[nodiscard]] inline auto get_timestamp_field(const PointCloud2 & msg)
{
  const auto field_it =
    std::find_if(msg.fields.cbegin(), msg.fields.cend(), [](const auto & field) {
      return (field.name == "t" || field.name == "timestamp" || field.name == "time");
    });

  if (field_it == msg.fields.cend()) {
    throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
  }

  return *field_it;
}

// Normalize timestamps from 0.0 to 1.0
[[nodiscard]] inline auto normalize_timestamps(const std::vector<double> & timestamps)
{
  const double max_timestamp = *std::max_element(timestamps.cbegin(), timestamps.cend());
  // check if already normalized
  if (max_timestamp < 1.0) {
    return timestamps;
  }
  std::vector<double> timestamps_normalized(timestamps.size());
  std::transform(
    timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
    [&](const auto & timestamp) { return timestamp / max_timestamp; });
  return timestamps_normalized;
}

[[nodiscard]] inline auto extract_timestamps_from_msg(
  const PointCloud2 & msg, const PointField & field)
{
  // Extract timestamps from cloud_msg
  const std::size_t n_points = msg.height * msg.width;
  std::vector<double> timestamps;
  timestamps.reserve(n_points);

  // Option 1: Timestamps are unsigned integers -> epoch time.
  if (field.name == "t" || field.name == "timestamp") {
    sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, field.name);
    for (std::size_t i = 0; i < n_points; ++i, ++msg_t) {
      timestamps.emplace_back(static_cast<double>(*msg_t));
    }
    // Covert to normalized time, between 0.0 and 1.0
    return normalize_timestamps(timestamps);
  }

  // Option 2: Timestamps are floating point values between 0.0 and 1.0
  // field.name == "timestamp"
  sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, field.name);
  for (std::size_t i = 0; i < n_points; ++i, ++msg_t) {
    timestamps.emplace_back(*msg_t);
  }
  return timestamps;
}

[[nodiscard]] inline auto create_point_cloud_2_msg(
  const std::size_t n_points, const Header & header, bool timestamp = false)
{
  PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  cloud_msg.header = header;
  cloud_msg.header.frame_id = fix_frame_id(cloud_msg.header.frame_id);
  cloud_msg.fields.clear();
  int offset = 0;
  offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
  offset += sizeOfPointField(PointField::FLOAT32);
  if (timestamp) {
    // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
    offset = addPointField(cloud_msg, "time", 1, PointField::FLOAT64, offset);
    offset += sizeOfPointField(PointField::FLOAT64);
  }

  // Resize the point cloud accordingly
  cloud_msg.point_step = offset;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
  modifier.resize(n_points);
  return cloud_msg;
}

inline void fill_point_cloud_2_xyz(const std::vector<Eigen::Vector3d> & points, PointCloud2 & msg)
{
  sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
  for (std::size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
    const Eigen::Vector3d & point = points[i];
    *msg_x = point.x();
    *msg_y = point.y();
    *msg_z = point.z();
  }
}

inline void fill_point_cloud_2_timestamp(const std::vector<double> & timestamps, PointCloud2 & msg)
{
  sensor_msgs::PointCloud2Iterator<double> msg_it(msg, "time");
  for (std::size_t i = 0; i < timestamps.size(); i++, ++msg_it) {
    *msg_it = timestamps[i];
  }
}

[[nodiscard]] inline std::vector<double> get_timestamps(const PointCloud2 & msg)
{
  return extract_timestamps_from_msg(msg, get_timestamp_field(msg));
}

[[nodiscard]] inline std::vector<Eigen::Vector3d> point_cloud_2_to_eigen(const PointCloud2 & msg)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(msg.height * msg.width);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");
  for (std::size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
  }
  return points;
}

[[nodiscard]] inline PointCloud2 eigen_to_point_cloud_2(
  const std::vector<Eigen::Vector3d> & points, const Header & header)
{
  auto msg = create_point_cloud_2_msg(points.size(), header);
  fill_point_cloud_2_xyz(points, msg);
  return msg;
}

[[nodiscard]] inline PointCloud2 eigen_to_point_cloud_2(
  const std::vector<Eigen::Vector3d> & points, const std::vector<double> & timestamps,
  const Header & header)
{
  constexpr auto kIncludeTimestamps = true;
  auto msg = create_point_cloud_2_msg(points.size(), header, kIncludeTimestamps);
  fill_point_cloud_2_xyz(points, msg);
  fill_point_cloud_2_timestamp(timestamps, msg);
  return msg;
}

template <typename T>
inline void load_field(const YAML::Node & yaml_node, T & config_field)
{
  config_field = yaml_node.as<T>();
}

[[nodiscard]] inline std::optional<kiss_icp::pipeline::KISSConfig> load_config(
  const std::string & file_path_str)
{
  kiss_icp::pipeline::KISSConfig config;

  try {
    const YAML::Node yaml_config = YAML::LoadFile(file_path_str);
    load_field(yaml_config["voxel_size"], config.voxel_size);
    load_field(yaml_config["max_range"], config.max_range);
    load_field(yaml_config["min_range"], config.min_range);
    load_field(yaml_config["max_points_per_voxel"], config.max_points_per_voxel);
    load_field(yaml_config["min_motion_th"], config.min_motion_th);
    load_field(yaml_config["initial_threshold"], config.initial_threshold);
    load_field(yaml_config["deskew"], config.deskew);
  } catch (const YAML::BadFile & e) {
    std::cerr << "Failed to parse Open3D odometry config (bad file): " << e.msg << "\n";
    return {};
  } catch (const YAML::ParserException & e) {
    std::cerr << "Failed to parse Open3D odometry config (parsing error): " << e.msg << "\n";
    return {};
  }

  return config;
}

inline std::ostream & operator<<(std::ostream & os, const kiss_icp::pipeline::KISSConfig & config)
{
  os << "KISS ICP Config: \n"
     << "\t Map - Voxel Size: " << config.voxel_size << "\n"
     << "\t Map - Max Range: " << config.max_range << "\n"
     << "\t Map - Min Range: " << config.min_range << "\n"
     << "\t Map - Max Points per Voxel: " << config.max_points_per_voxel << "\n"
     << "\t Threshold - Min Motion Threshold: " << config.min_motion_th << "\n"
     << "\t Threshold - Initial Threshold: " << config.initial_threshold << "\n"
     << "\t Motion Compensation - Deskew: " << config.deskew << "\n";
  return os;
}

}  // namespace r2k_odom_kiss_icp

#endif  // ROS2_KITTI_ODOM_KISS_ICP__UTILS_HPP_
