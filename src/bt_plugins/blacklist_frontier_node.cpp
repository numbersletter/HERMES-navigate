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

#include "hermes_navigate/bt_plugins/blacklist_frontier_node.hpp"

#include <vector>

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

BlacklistFrontierNode::BlacklistFrontierNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList BlacklistFrontierNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("blacklisted_goals"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus BlacklistFrontierNode::tick()
{
  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_res) {
    // Nothing to blacklist; this should not normally happen.
    return BT::NodeStatus::SUCCESS;
  }

  // Read the current blacklist (empty if not yet initialised).
  auto bl_res = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("blacklisted_goals");
  std::vector<geometry_msgs::msg::PoseStamped> blacklist =
    bl_res ? bl_res.value() : std::vector<geometry_msgs::msg::PoseStamped>{};

  const auto & new_goal = goal_res.value();

  // Deduplicate: skip appending if an entry within 1 cm already exists.
  // This tiny radius is intentionally strict — it only guards against the
  // exact same numerical position being added twice (e.g., if the BT node is
  // re-ticked on the same failed goal before the frontier list refreshes).
  // Use the configurable blacklist_radius from SelectFrontierNode if broader
  // deduplication is needed.
  constexpr double kDupRadiusSq = 0.01 * 0.01;  // (1 cm)^2
  for (const auto & existing : blacklist) {
    const double dx = new_goal.pose.position.x - existing.pose.position.x;
    const double dy = new_goal.pose.position.y - existing.pose.position.y;
    if ((dx * dx + dy * dy) < kDupRadiusSq) {
      return BT::NodeStatus::SUCCESS;  // already blacklisted
    }
  }

  // Append the failed frontier and write the updated list back.
  blacklist.push_back(new_goal);
  setOutput("blacklisted_goals", blacklist);

  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void BlacklistFrontierNode::registerWithFactory(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<BlacklistFrontierNode>("BlacklistFrontier");
}

}  // namespace hermes_navigate
