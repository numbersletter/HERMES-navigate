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

#include "hermes_navigate/should_do_coverage.hpp"

#include <algorithm>
#include <cmath>

namespace hermes_navigate
{

ShouldDoCoverage::ShouldDoCoverage(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  // coverage_period_ is derived from coverage_task_ratio in tick() so that
  // blackboard / XML changes take effect dynamically.  The value set here is
  // only used if the port is not connected (unlikely in production but safe).
  coverage_period_ = 5;  // corresponds to the default ratio of 0.2
}

BT::PortsList ShouldDoCoverage::providedPorts()
{
  // coverage_task_ratio is read from the BT XML / blackboard at construction
  return {BT::InputPort<double>("coverage_task_ratio", 0.2,
      "Fraction of iterations dedicated to coverage (0–1).")};
}

BT::NodeStatus ShouldDoCoverage::tick()
{
  // Re-read ratio every tick (allows dynamic reconfiguration via blackboard)
  auto ratio_res = getInput<double>("coverage_task_ratio");
  if (ratio_res) {
    double r = std::max(0.0, std::min(1.0, ratio_res.value()));
    if (r > 0.0) {
      coverage_period_ = static_cast<int>(std::round(1.0 / r));
    } else {
      coverage_period_ = std::numeric_limits<int>::max();  // never do coverage
    }
  }

  ++step_count_;
  bool do_coverage = (coverage_period_ > 0 && (step_count_ % coverage_period_) == 0);
  return do_coverage ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace hermes_navigate

// ─── Plugin registration ──────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::ShouldDoCoverage>("ShouldDoCoverage");
}
