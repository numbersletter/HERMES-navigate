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

#include "hermes_navigate/bt_plugins/return_to_start_condition.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ReturnToStartCondition::ReturnToStartCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList ReturnToStartCondition::providedPorts()
{
  return {BT::InputPort<bool>("return_to_start")};
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus ReturnToStartCondition::tick()
{
  auto res = getInput<bool>("return_to_start");
  if (!res) {
    throw BT::RuntimeError(
      "ReturnToStartCondition: missing required port 'return_to_start': ", res.error());
  }
  return res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void ReturnToStartCondition::registerWithFactory(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ReturnToStartCondition>("ReturnToStartCondition");
}

}  // namespace hermes_navigate
