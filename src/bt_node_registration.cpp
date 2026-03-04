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

// Single registration point for all HERMES BT nodes.
// All five node types are compiled into one shared library
// (hermes_navigate_bt_plugins), so there must be exactly one
// BT_REGISTER_NODES / BT_RegisterNodesFromPlugin symbol per library.
// Individual source files must NOT define BT_REGISTER_NODES.

#include "behaviortree_cpp/bt_factory.h"

#include "hermes_navigate/compute_frontiers.hpp"
#include "hermes_navigate/score_frontiers.hpp"
#include "hermes_navigate/select_next_frontier.hpp"
#include "hermes_navigate/should_do_coverage.hpp"
#include "hermes_navigate/select_coverage_waypoint.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::ComputeFrontiers>("ComputeFrontiers");
  factory.registerNodeType<hermes_navigate::ScoreFrontiers>("ScoreFrontiers");
  factory.registerNodeType<hermes_navigate::SelectNextFrontier>("SelectNextFrontier");
  factory.registerNodeType<hermes_navigate::ShouldDoCoverage>("ShouldDoCoverage");
  factory.registerNodeType<hermes_navigate::SelectCoverageWaypoint>("SelectCoverageWaypoint");
}
