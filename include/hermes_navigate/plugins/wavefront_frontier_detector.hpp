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

#ifndef HERMES_NAVIGATE__PLUGINS__WAVEFRONT_FRONTIER_DETECTOR_HPP_
#define HERMES_NAVIGATE__PLUGINS__WAVEFRONT_FRONTIER_DETECTOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include "hermes_navigate/plugins/base_frontier_search.hpp"
#include "hermes_navigate/costmap_constants.hpp"

namespace hermes_navigate
{

/**
 * @class WavefrontFrontierDetector
 * @brief Incremental wavefront (BFS-based) frontier detection algorithm.
 *
 * Implements BaseFrontierSearch via a two-phase BFS:
 *   1. Map-Open BFS: Seeds from robot position, expands through free cells,
 *      labels reachable free cells as "map-open".
 *   2. Frontier labelling: Any map-open cell adjacent to an unknown cell is
 *      a frontier cell.
 *   3. Clustering: Adjacent frontier cells are grouped into clusters.
 *      Small clusters (< min_frontier_size cells) are discarded.
 *
 * This is the "Wavefront Frontier Detector" (WFD) as described in:
 *   Keidar & Kaminka, IROS 2012.
 *
 * Runtime-configurable parameters (in params/plugin_params.yaml):
 *   wavefront_frontier_detector.min_frontier_size   (int,    default 3)
 *   wavefront_frontier_detector.clustering_distance  (double, default 0.5 m)
 *   wavefront_frontier_detector.search_distance      (double, default 10.0 m, 0 = unlimited)
 */
class WavefrontFrontierDetector : public BaseFrontierSearch
{
public:
  WavefrontFrontierDetector() = default;
  ~WavefrontFrontierDetector() override = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name) override;

  std::vector<Frontier> searchFrontiers(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("WavefrontFrontierDetector")};

  // Runtime parameters
  int min_frontier_size_{3};
  double clustering_distance_{0.5};
  double search_distance_{10.0};  ///< Max search radius from robot (m); 0 = unlimited.

  /// @brief BFS from robot position to label reachable free cells,
  ///        then identify frontier cells (free + unknown-adjacent).
  std::vector<std::pair<int, int>> detectFrontierCells(
    const nav2_msgs::msg::Costmap & costmap,
    int robot_gx, int robot_gy);

  /// @brief Group frontier cells into clusters via BFS within clustering_distance_.
  std::vector<std::vector<std::pair<int, int>>> clusterFrontiers(
    const std::vector<std::pair<int, int>> & cells,
    const nav2_msgs::msg::Costmap & costmap);

  /// @brief Convert a cluster to a Frontier struct (centroid + goal pose).
  Frontier clusterToFrontier(
    const std::vector<std::pair<int, int>> & cluster,
    const nav2_msgs::msg::Costmap & costmap);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__PLUGINS__WAVEFRONT_FRONTIER_DETECTOR_HPP_
