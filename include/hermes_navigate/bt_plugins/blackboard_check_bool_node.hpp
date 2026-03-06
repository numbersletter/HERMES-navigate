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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__BLACKBOARD_CHECK_BOOL_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__BLACKBOARD_CHECK_BOOL_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace hermes_navigate
{

/**
 * @class BlackboardCheckBool
 * @brief BT condition node that compares two bool values read from the blackboard.
 *
 * Returns SUCCESS when value_A == value_B; otherwise returns the status
 * specified by return_on_mismatch (default: FAILURE).
 *
 * BT ports:
 *   Input: "value_A"            — bool  (typically a blackboard entry)
 *   Input: "value_B"            — bool  (typically a literal "true"/"false")
 *   Input: "return_on_mismatch" — string "SUCCESS" or "FAILURE" (default: "FAILURE")
 */
class BlackboardCheckBool : public BT::SyncActionNode
{
public:
  BlackboardCheckBool(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(BT::BehaviorTreeFactory & factory);

  BT::NodeStatus tick() override;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__BLACKBOARD_CHECK_BOOL_NODE_HPP_
