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

#ifndef HERMES_NAVIGATE__FRONTIER_TYPES_HPP_
#define HERMES_NAVIGATE__FRONTIER_TYPES_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hermes_navigate
{

/// @brief Represents a detected frontier cluster in the occupancy grid.
struct Frontier
{
  double centroid_x{0.0};    ///< World-frame X of the cluster centroid.
  double centroid_y{0.0};    ///< World-frame Y of the cluster centroid.
  int size{0};               ///< Number of frontier cells in the cluster.
  geometry_msgs::msg::PoseStamped goal_pose;  ///< Nav2-ready goal at the centroid.
};

/// @brief A Frontier with an associated composite score (higher = more attractive).
struct ScoredFrontier
{
  Frontier frontier;
  double score{0.0};
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__FRONTIER_TYPES_HPP_
