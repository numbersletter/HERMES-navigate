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

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ReturnToStartCondition::ReturnToStartCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Logger logger)
: BT::ConditionNode(name, config),
  logger_(logger)
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
  if (res.value()) {
    RCLCPP_INFO(logger_,
      "ReturnToStartCondition: return_to_start=true — activating return-to-start branch.");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void ReturnToStartCondition::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Logger logger)
{
  factory.registerBuilder<ReturnToStartCondition>(
    "ReturnToStartCondition",
    [logger](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReturnToStartCondition>(name, config, logger);
    });
}

}  // namespace hermes_navigate
