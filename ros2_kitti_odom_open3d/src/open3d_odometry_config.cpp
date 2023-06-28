#include "ros2_kitti_odom_open3d/open3d_odometry_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>

namespace r2k_odom_o3d
{

ICPIterationSettings::ICPIterationSettings(
  const double max_correspondence_distance_in, const double relative_fitness,
  const double relative_rmse, const int max_iteration)
: max_correspondence_distance(max_correspondence_distance_in),
  convergence_criteria{relative_fitness, relative_rmse, max_iteration}
{
}

NormalComputationSettings::NormalComputationSettings(
  const bool fast_normal_computation_in, const int max_nn_in, const double radius_in)
: fast_normal_computation(fast_normal_computation_in), max_nn(max_nn_in), radius(radius_in)
{
}

O3DICPConfig::O3DICPConfig(
  const float decimation_factor_in, const NormalComputationSettings & normal_computation_in,
  const std::vector<ICPIterationSettings> & iterations_in)
: decimation_factor(decimation_factor_in),
  normal_computation(normal_computation_in),
  iterations(iterations_in)
{
}

template <typename T>
void load_field(const YAML::Node & yaml_node, T & config_field)
{
  config_field = yaml_node.as<T>();
}

std::optional<O3DICPConfig> load_config(const std::string & file_path_str)
{
  if (const std::filesystem::path file_path{file_path_str};
      !std::filesystem::exists(file_path) ||
      file_path.extension().string() != std::string{".yml"}) {
    std::cerr << "Failed to load Open3D odometry config file: " << file_path_str
              << " - Could not find file or invalid extension\n";
    return {};
  }

  O3DICPConfig config;
  config.iterations.clear();

  try {
    const YAML::Node yaml_config = YAML::LoadFile(file_path_str);

    load_field(yaml_config["decimation_factor"], config.decimation_factor);

    const auto normal_node = yaml_config["normal_computation"];
    auto & normal_field = config.normal_computation;
    load_field(normal_node["fast_normal_computation"], normal_field.fast_normal_computation);
    load_field(normal_node["max_nn"], normal_field.max_nn);
    load_field(normal_node["radius"], normal_field.radius);

    const auto iter_nodes = yaml_config["iterations"];
    if (iter_nodes.size() == 0) {
      std::cerr << "Failed to parse Open3D odometry config: No iteration settings specified\n";
      return {};
    }

    for (const auto & iter_node : iter_nodes) {
      ICPIterationSettings setting;
      load_field(normal_node["max_correspondence_distance"], setting.max_correspondence_distance);
      load_field(normal_node["relative_fitness"], setting.convergence_criteria.relative_fitness_);
      load_field(normal_node["relative_rmse"], setting.convergence_criteria.relative_rmse_);
      load_field(normal_node["max_iteration"], setting.convergence_criteria.max_iteration_);
      config.iterations.push_back(setting);
    }
  } catch (const YAML::BadFile & e) {
    std::cerr << "Failed to parse Open3D odometry config (bad file): " << e.msg << "\n";
    return {};
  } catch (const YAML::ParserException & e) {
    std::cerr << "Failed to parse Open3D odometry config (parsing error): " << e.msg << "\n";
    return {};
  }

  return config;
}

}  // namespace r2k_odom_o3d
