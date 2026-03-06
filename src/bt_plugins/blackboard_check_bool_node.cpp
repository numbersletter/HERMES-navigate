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

#include "hermes_navigate/bt_plugins/blackboard_check_bool_node.hpp"

#include <string>

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

BlackboardCheckBool::BlackboardCheckBool(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList BlackboardCheckBool::providedPorts()
{
  return {
    BT::InputPort<bool>("value_A"),
    BT::InputPort<bool>("value_B"),
    BT::InputPort<std::string>("return_on_mismatch", "FAILURE",
      "NodeStatus to return when value_A != value_B ('SUCCESS' or 'FAILURE')")
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus BlackboardCheckBool::tick()
{
  auto a_res = getInput<bool>("value_A");
  if (!a_res) {
    throw BT::RuntimeError("BlackboardCheckBool: missing required port 'value_A': ",
      a_res.error());
  }

  auto b_res = getInput<bool>("value_B");
  if (!b_res) {
    throw BT::RuntimeError("BlackboardCheckBool: missing required port 'value_B': ",
      b_res.error());
  }

  if (a_res.value() == b_res.value()) {
    return BT::NodeStatus::SUCCESS;
  }

  auto mismatch = getInput<std::string>("return_on_mismatch");
  const std::string mismatch_str = mismatch ? mismatch.value() : "FAILURE";

  if (mismatch_str == "SUCCESS") {
    return BT::NodeStatus::SUCCESS;
  }
  if (mismatch_str != "FAILURE") {
    throw BT::RuntimeError(
      "BlackboardCheckBool: 'return_on_mismatch' must be 'SUCCESS' or 'FAILURE', got: ",
      mismatch_str);
  }
  return BT::NodeStatus::FAILURE;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void BlackboardCheckBool::registerWithFactory(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<BlackboardCheckBool>("BlackboardCheckBool");
}

}  // namespace hermes_navigate
