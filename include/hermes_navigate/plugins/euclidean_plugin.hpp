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

#ifndef HERMES_NAVIGATE__PLUGINS__EUCLIDEAN_PLUGIN_HPP_
#define HERMES_NAVIGATE__PLUGINS__EUCLIDEAN_PLUGIN_HPP_

#include <string>

#include "hermes_navigate/plugins/base_cost_plugin.hpp"

namespace hermes_navigate
{

/**
 * @class EuclideanPlugin
 * @brief Scores frontiers by Euclidean distance from the robot (lower = better).
 *
 * Returns `weight * (-distance)` so that closer frontiers receive a higher
 * (less negative) contribution to the composite score.
 *
 * Runtime-configurable parameters (in params/plugin_params.yaml):
 *   euclidean.weight  (double, default -0.5)  — negative to penalise distance
 */
class EuclideanPlugin : public BaseCostPlugin
{
public:
  EuclideanPlugin() = default;
  ~EuclideanPlugin() override = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name) override;

  double scoreFrontier(
    const Frontier & frontier,
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("EuclideanPlugin")};

  /// Negative weight: multiplied by Euclidean distance, so larger distance →
  /// more-negative score contribution.
  double weight_{-0.5};
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__PLUGINS__EUCLIDEAN_PLUGIN_HPP_
