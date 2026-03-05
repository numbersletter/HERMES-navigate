// Copyright 2024 HERMES Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hermes_navigate/plugins/information_gain_plugin.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace hermes_navigate
{

// ─── Initialization ───────────────────────────────────────────────────────────

void InformationGainPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & name)
{
  auto node = parent.lock();
  if (!node) {
    return;
  }

  logger_ = node->get_logger();

  const std::string prefix = name + ".";
  node->declare_parameter(prefix + "weight",           rclcpp::ParameterValue(1.0));
  node->declare_parameter(prefix + "max_ray_length_m", rclcpp::ParameterValue(5.0));
  node->declare_parameter(prefix + "num_rays",         rclcpp::ParameterValue(36));

  weight_           = node->get_parameter(prefix + "weight").as_double();
  max_ray_length_m_ = node->get_parameter(prefix + "max_ray_length_m").as_double();
  num_rays_         = node->get_parameter(prefix + "num_rays").as_int();

  RCLCPP_INFO(logger_,
    "InformationGainPlugin initialised [weight=%.2f, ray_len=%.2f m, rays=%d]",
    weight_, max_ray_length_m_, num_rays_);
}

// ─── Scoring ─────────────────────────────────────────────────────────────────

double InformationGainPlugin::scoreFrontier(
  const Frontier & frontier,
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/)
{
  const double res = costmap.metadata.resolution;
  const double ox  = costmap.metadata.origin.position.x;
  const double oy  = costmap.metadata.origin.position.y;
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);

  // Frontier centroid in grid coordinates
  int fx = static_cast<int>((frontier.centroid_x - ox) / res);
  int fy = static_cast<int>((frontier.centroid_y - oy) / res);

  if (fx < 0 || fx >= width || fy < 0 || fy >= height) {
    return 0.0;
  }

  const int max_steps = static_cast<int>(max_ray_length_m_ / res);

  // Cast rays radially and count unknown cells visible through them
  int total_unknown = 0;
  const double angle_step = 2.0 * M_PI / static_cast<double>(num_rays_);

  for (int r = 0; r < num_rays_; ++r) {
    double angle = r * angle_step;
    double dir_x = std::cos(angle);
    double dir_y = std::sin(angle);
    total_unknown += castRay(costmap, fx, fy, dir_x, dir_y, max_steps);
  }

  return weight_ * static_cast<double>(total_unknown);
}

// ─── Ray casting ─────────────────────────────────────────────────────────────

int InformationGainPlugin::castRay(
  const nav2_msgs::msg::Costmap & costmap,
  int start_gx, int start_gy,
  double dir_x, double dir_y,
  int max_steps) const
{
  using namespace hermes_navigate::costmap;  // NOLINT(build/namespaces)

  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const auto & data = costmap.data;

  int unknown_count = 0;

  // Bresenham-style DDA ray march
  double px = static_cast<double>(start_gx) + 0.5;
  double py = static_cast<double>(start_gy) + 0.5;

  for (int step = 0; step < max_steps; ++step) {
    px += dir_x;
    py += dir_y;

    int gx = static_cast<int>(px);
    int gy = static_cast<int>(py);

    if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
      break;
    }

    uint8_t cost = data[static_cast<std::size_t>(gy * width + gx)];

    if (cost == UNKNOWN) {
      ++unknown_count;
    } else if (cost >= LETHAL) {
      // Ray blocked by lethal obstacle
      break;
    }
    // Free cells: ray continues
  }

  return unknown_count;
}

}  // namespace hermes_navigate

// ─── Plugin export ────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  hermes_navigate::InformationGainPlugin,
  hermes_navigate::BaseCostPlugin)
