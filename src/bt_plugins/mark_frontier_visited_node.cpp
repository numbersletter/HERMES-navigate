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

#include "hermes_navigate/bt_plugins/mark_frontier_visited_node.hpp"

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

MarkFrontierVisitedNode::MarkFrontierVisitedNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Logger logger)
: BT::SyncActionNode(name, config),
  logger_(logger)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList MarkFrontierVisitedNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("visited_frontiers"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus MarkFrontierVisitedNode::tick()
{
  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_res) {
    // Nothing to mark; this should not normally happen.
    RCLCPP_WARN(logger_,
      "MarkFrontierVisitedNode: 'goal' port not set — nothing to mark visited.");
    return BT::NodeStatus::SUCCESS;
  }

  // Read the current visited list (empty if not yet initialised).
  auto vf_res =
    getInput<std::vector<geometry_msgs::msg::PoseStamped>>("visited_frontiers");
  std::vector<geometry_msgs::msg::PoseStamped> visited =
    vf_res ? vf_res.value() : std::vector<geometry_msgs::msg::PoseStamped>{};

  const auto & new_goal = goal_res.value();

  // Deduplicate: skip appending if an entry within 1 cm already exists.
  // This tiny radius only guards against the exact same numerical position
  // being added twice (e.g., the same frontier is re-ticked before the
  // frontier list refreshes).  The broader spatial exclusion is handled by
  // SelectFrontierNode using the blacklist_radius_m parameter.
  constexpr double kDupRadiusSq = 0.01 * 0.01;  // (1 cm)^2
  for (const auto & existing : visited) {
    const double dx = new_goal.pose.position.x - existing.pose.position.x;
    const double dy = new_goal.pose.position.y - existing.pose.position.y;
    if ((dx * dx + dy * dy) < kDupRadiusSq) {
      RCLCPP_DEBUG(logger_,
        "MarkFrontierVisitedNode: goal (%.2f, %.2f) already in visited list — "
        "skipping duplicate.",
        new_goal.pose.position.x, new_goal.pose.position.y);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Append the frontier and write the updated list back.
  RCLCPP_INFO(logger_,
    "MarkFrontierVisitedNode: frontier at (%.2f, %.2f) successfully reached — "
    "marked as visited [visited list now has %zu entries].",
    new_goal.pose.position.x, new_goal.pose.position.y,
    visited.size() + 1);

  visited.push_back(new_goal);
  setOutput("visited_frontiers", visited);

  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void MarkFrontierVisitedNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Logger logger)
{
  factory.registerBuilder<MarkFrontierVisitedNode>(
    "MarkFrontierVisited",
    [logger](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MarkFrontierVisitedNode>(name, config, logger);
    });
}

}  // namespace hermes_navigate
