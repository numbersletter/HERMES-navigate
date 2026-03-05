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

#ifndef HERMES_NAVIGATE__PLUGINS__BASE_COST_PLUGIN_HPP_
#define HERMES_NAVIGATE__PLUGINS__BASE_COST_PLUGIN_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "hermes_navigate/frontier_types.hpp"

namespace hermes_navigate
{

/**
 * @class BaseCostPlugin
 * @brief Abstract pluginlib base class for frontier cost/scoring plugins.
 *
 * Each derived class computes a scalar contribution to the overall frontier
 * score.  The AssignCostsNode BT node loads a list of these plugins and
 * accumulates their scores for each frontier candidate.
 *
 * Positive scores attract the robot toward the frontier; negative values
 * repel.  It is the responsibility of each plugin to weight its contribution
 * appropriately (see plugin_params.yaml for per-plugin weight parameters).
 *
 * Runtime-configurable parameters for each derived class are declared in
 * params/plugin_params.yaml and read from the parent lifecycle node.
 */
class BaseCostPlugin
{
public:
  using SharedPtr = std::shared_ptr<BaseCostPlugin>;

  virtual ~BaseCostPlugin() = default;

  /**
   * @brief Initialize the plugin.
   * @param parent  Weak pointer to the parent lifecycle node (context node).
   * @param name    Plugin instance name (parameter namespace prefix).
   */
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name) = 0;

  /**
   * @brief Score a single frontier candidate.
   * @param frontier   The frontier to score.
   * @param costmap    Current global costmap.
   * @param robot_pose Current robot pose in the map frame.
   * @return           Scalar score contribution (positive = attractive).
   */
  virtual double scoreFrontier(
    const Frontier & frontier,
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose) = 0;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__PLUGINS__BASE_COST_PLUGIN_HPP_
