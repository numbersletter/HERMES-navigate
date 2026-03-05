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

#ifndef HERMES_NAVIGATE__PLUGINS__BASE_FRONTIER_SEARCH_HPP_
#define HERMES_NAVIGATE__PLUGINS__BASE_FRONTIER_SEARCH_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "hermes_navigate/frontier_types.hpp"

namespace hermes_navigate
{

/**
 * @class BaseFrontierSearch
 * @brief Abstract pluginlib base class for frontier search algorithms.
 *
 * All frontier detection algorithms (e.g., WavefrontFrontierDetector) must
 * inherit from this class and be exported via PLUGINLIB_EXPORT_CLASS so that
 * the SearchFrontiersNode BT node can load them at runtime.
 *
 * Runtime-configurable parameters for each derived class are declared in
 * params/plugin_params.yaml and read from the parent lifecycle node.
 */
class BaseFrontierSearch
{
public:
  using SharedPtr = std::shared_ptr<BaseFrontierSearch>;

  virtual ~BaseFrontierSearch() = default;

  /**
   * @brief Initialize the plugin.
   * @param parent  Weak pointer to the parent lifecycle node (context node).
   *                Used to access parameters and the logger.
   * @param name    Plugin instance name (used as parameter namespace prefix).
   */
  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name) = 0;

  /**
   * @brief Search the costmap for frontier candidates.
   * @param costmap    Latest global costmap from Nav2.
   * @param robot_pose Current robot pose in the map frame.
   * @return           Detected and clustered frontier list.
   */
  virtual std::vector<Frontier> searchFrontiers(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose) = 0;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__PLUGINS__BASE_FRONTIER_SEARCH_HPP_
