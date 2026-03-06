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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__SEARCH_FRONTIERS_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__SEARCH_FRONTIERS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "hermes_navigate/frontier_types.hpp"
#include "hermes_navigate/plugins/base_frontier_search.hpp"

namespace hermes_navigate
{

/**
 * @class SearchFrontiersNode
 * @brief BT action node that searches for exploration frontiers using a
 *        pluginlib-loaded BaseFrontierSearch implementation.
 *
 * The comprising plugin is selected at runtime via the
 * `frontier_search.plugin` parameter on the parent lifecycle node.
 * Default plugin: "hermes_navigate::WavefrontFrontierDetector".
 *
 * Publishes a MarkerArray to /hermes/frontiers for visualisation.
 *
 * BT ports:
 *   Input:  "robot_pose"  — geometry_msgs::msg::PoseStamped (current robot pose)
 *   Output: "frontiers"   — std::vector<Frontier>
 */
class SearchFrontiersNode : public BT::ActionNodeBase
{
public:
  /**
   * @param name    BT node name.
   * @param config  BT node config (ports).
   * @param parent  Weak pointer to the HermesNavigateNode (context node).
   *                Used to access parameters and create publishers.
   */
  SearchFrontiersNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(
    BT::BehaviorTreeFactory & factory,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  BT::NodeStatus tick() override;

  void halt() override {}

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  // Costmap subscription (spun on the parent node's executor)
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Frontier search plugin
  pluginlib::ClassLoader<BaseFrontierSearch> plugin_loader_;
  BaseFrontierSearch::SharedPtr frontier_plugin_;

  // Visualisation
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  /// @brief Publish frontiers as RViz sphere markers.
  void publishMarkers(const std::vector<Frontier> & frontiers);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__SEARCH_FRONTIERS_NODE_HPP_
