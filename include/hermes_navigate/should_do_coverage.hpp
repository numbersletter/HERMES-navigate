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

#ifndef HERMES_NAVIGATE__SHOULD_DO_COVERAGE_HPP_
#define HERMES_NAVIGATE__SHOULD_DO_COVERAGE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

/**
 * @class ShouldDoCoverage
 * @brief BT condition node that decides whether the robot should execute a
 *        camera-coverage task on this iteration rather than exploration.
 *
 * Returns SUCCESS (→ trigger coverage branch) when the internal counter
 * indicates it is time for a coverage step, based on `coverage_task_ratio`.
 * For example, with ratio 0.2 the node succeeds on approximately every 5th
 * tick, measured as: (step_count % round(1 / ratio)) == 0.
 *
 * BT ports:  none (stateful counter maintained internally).
 */
class ShouldDoCoverage : public BT::ConditionNode
{
public:
  ShouldDoCoverage(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  int step_count_{0};
  int coverage_period_{5};   ///< Derived from coverage_task_ratio.
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__SHOULD_DO_COVERAGE_HPP_
