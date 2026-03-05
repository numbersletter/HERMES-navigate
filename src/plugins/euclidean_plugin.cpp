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

#include "hermes_navigate/plugins/euclidean_plugin.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace hermes_navigate
{

// ─── Initialization ───────────────────────────────────────────────────────────

void EuclideanPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & name)
{
  auto node = parent.lock();
  if (!node) {
    return;
  }

  logger_ = node->get_logger();

  const std::string prefix = name + ".";
  node->declare_parameter(prefix + "weight", rclcpp::ParameterValue(-0.5));

  weight_ = node->get_parameter(prefix + "weight").as_double();

  RCLCPP_INFO(logger_, "EuclideanPlugin initialised [weight=%.2f]", weight_);
}

// ─── Scoring ─────────────────────────────────────────────────────────────────

double EuclideanPlugin::scoreFrontier(
  const Frontier & frontier,
  const nav2_msgs::msg::Costmap & /*costmap*/,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  double dx = frontier.centroid_x - robot_pose.pose.position.x;
  double dy = frontier.centroid_y - robot_pose.pose.position.y;
  double dist = std::hypot(dx, dy);

  // weight_ is typically negative so that larger distance → more-negative score.
  return weight_ * dist;
}

}  // namespace hermes_navigate

// ─── Plugin export ────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  hermes_navigate::EuclideanPlugin,
  hermes_navigate::BaseCostPlugin)
