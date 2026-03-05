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

#ifndef HERMES_NAVIGATE__PLUGINS__INFORMATION_GAIN_PLUGIN_HPP_
#define HERMES_NAVIGATE__PLUGINS__INFORMATION_GAIN_PLUGIN_HPP_

#include <string>

#include "hermes_navigate/plugins/base_cost_plugin.hpp"
#include "hermes_navigate/costmap_constants.hpp"

namespace hermes_navigate
{

/**
 * @class InformationGainPlugin
 * @brief Scores frontiers by the number of unknown cells visible via ray-casting.
 *
 * For each frontier, rays are cast radially from the frontier centroid up to
 * max_ray_length_m.  Each ray counts the number of unknown costmap cells it
 * passes through before hitting a lethal obstacle or the map boundary.  The
 * total count (weighted by `weight`) is the score contribution.
 *
 * Using ray-casting rather than a simple circular radius more accurately
 * models what cells a sensor could actually observe from that position.
 *
 * Runtime-configurable parameters (in params/plugin_params.yaml):
 *   information_gain.weight           (double, default  1.0)  — score multiplier
 *   information_gain.max_ray_length_m (double, default  5.0)  — ray length (m)
 *   information_gain.num_rays         (int,    default 36)     — angular resolution
 */
class InformationGainPlugin : public BaseCostPlugin
{
public:
  InformationGainPlugin() = default;
  ~InformationGainPlugin() override = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name) override;

  double scoreFrontier(
    const Frontier & frontier,
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("InformationGainPlugin")};

  double weight_{1.0};
  double max_ray_length_m_{5.0};
  int num_rays_{36};

  /// @brief Cast a single ray from (start_gx, start_gy) in direction (dx, dy).
  /// @return Number of unknown cells encountered before obstruction.
  int castRay(
    const nav2_msgs::msg::Costmap & costmap,
    int start_gx, int start_gy,
    double dir_x, double dir_y,
    int max_steps) const;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__PLUGINS__INFORMATION_GAIN_PLUGIN_HPP_
