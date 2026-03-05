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

/**
 * @file register_nodes.cpp
 * @brief Central BT node registration for the hermes_navigate_bt_plugins library.
 *
 * This file provides the single BT_REGISTER_NODES entry point that registers
 * all BT action/condition node types provided by this package.  Placing all
 * registrations here avoids duplicate-symbol linker errors when multiple BT
 * node implementations are compiled into one shared library.
 *
 * Note: The new SearchFrontiersNode / AssignCostsNode / SelectFrontierNode /
 * ReturnToStartNode nodes require a parent lifecycle-node pointer and are
 * therefore registered by HermesNavigateNode::registerBTNodes() at runtime
 * (see hermes_navigate_node.cpp), not here.  This file registers the
 * standalone nodes that do not need that context.
 */

#include "behaviortree_cpp/bt_factory.h"

#include "hermes_navigate/bt_plugins/select_frontier_node.hpp"
// Legacy nodes (kept for backward compatibility with explore_and_cover.xml)
#include "hermes_navigate/should_do_coverage.hpp"
#include "hermes_navigate/select_coverage_waypoint.hpp"
#include "hermes_navigate/compute_frontiers.hpp"
#include "hermes_navigate/score_frontiers.hpp"
#include "hermes_navigate/select_next_frontier.hpp"

BT_REGISTER_NODES(factory)
{
  // ── Legacy exploration nodes (used by explore_and_cover.xml) ──────────────
  factory.registerNodeType<hermes_navigate::ComputeFrontiers>("ComputeFrontiers");
  factory.registerNodeType<hermes_navigate::ScoreFrontiers>("ScoreFrontiers");
  factory.registerNodeType<hermes_navigate::SelectNextFrontier>("SelectNextFrontier");
  factory.registerNodeType<hermes_navigate::ShouldDoCoverage>("ShouldDoCoverage");
  factory.registerNodeType<hermes_navigate::SelectCoverageWaypoint>("SelectCoverageWaypoint");
}
