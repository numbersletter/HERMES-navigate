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

#include "hermes_navigate/bt_plugins/navigate_to_frontier_node.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

NavigateToFrontierNode::NavigateToFrontierNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::SyncActionNode(name, config),
  parent_(parent)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList NavigateToFrontierNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus NavigateToFrontierNode::tick()
{
  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_res) {
    auto node = parent_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "NavigateToFrontierNode: no frontier goal available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  const auto & goal = goal_res.value();

  // Write the selected frontier's goal pose to the nav_goal blackboard entry
  // so that the downstream NavigateToPose BT node (provided by Nav2) can
  // dispatch the navigate_to_pose action request.
  setOutput("nav_goal", goal);

  auto node = parent_.lock();
  if (node) {
    RCLCPP_DEBUG(node->get_logger(),
      "NavigateToFrontierNode: frontier goal set at (%.2f, %.2f).",
      goal.pose.position.x, goal.pose.position.y);
  }

  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void NavigateToFrontierNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<NavigateToFrontierNode>(
    "NavigateToFrontier",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToFrontierNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
