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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__ASSIGN_COSTS_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__ASSIGN_COSTS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "hermes_navigate/frontier_types.hpp"
#include "hermes_navigate/plugins/base_cost_plugin.hpp"

namespace hermes_navigate
{

/**
 * @class AssignCostsNode
 * @brief BT action node that scores frontier candidates using a list of
 *        pluginlib-loaded BaseCostPlugin implementations.
 *
 * The comprising cost plugins are selected at runtime via the
 * `cost_plugins` parameter (list of plugin type names) on the parent
 * lifecycle node.  Default plugins:
 *   - "hermes_navigate::InformationGainPlugin"
 *   - "hermes_navigate::EuclideanPlugin"
 *
 * Each plugin contributes an additive score; the combined score determines
 * which frontier the robot should visit next.
 *
 * Publishes a MarkerArray to /hermes/scored_frontiers for visualisation.
 *
 * BT ports:
 *   Input:  "frontiers"         — std::vector<Frontier>
 *   Input:  "robot_pose"        — geometry_msgs::msg::PoseStamped
 *   Output: "scored_frontiers"  — std::vector<ScoredFrontier>
 */
class AssignCostsNode : public BT::SyncActionNode
{
public:
  AssignCostsNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  // Costmap subscription
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Cost plugins
  pluginlib::ClassLoader<BaseCostPlugin> plugin_loader_;
  std::vector<BaseCostPlugin::SharedPtr> cost_plugins_;
  std::vector<std::string> plugin_names_;  ///< Names of loaded plugins (for logging).

  // Visualisation
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  /// @brief Publish scored frontiers as text markers showing their score.
  void publishMarkers(const std::vector<ScoredFrontier> & scored);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__ASSIGN_COSTS_NODE_HPP_
