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

#include "hermes_navigate/compute_frontiers.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hermes_navigate
{

// ─── BT node registration ────────────────────────────────────────────────────

ComputeFrontiers::ComputeFrontiers(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("compute_frontiers_node");

  // Parameters with defaults
  min_frontier_size_   = 3.0;
  clustering_distance_ = 0.5;

  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      latest_costmap_ = msg;
    });
}

BT::PortsList ComputeFrontiers::providedPorts()
{
  return {BT::OutputPort<std::vector<Frontier>>("frontiers")};
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus ComputeFrontiers::tick()
{
  // Spin once to receive the costmap message if available
  rclcpp::spin_some(node_);

  if (!latest_costmap_) {
    RCLCPP_WARN(node_->get_logger(), "ComputeFrontiers: no costmap received yet.");
    return BT::NodeStatus::FAILURE;
  }

  const auto & costmap = *latest_costmap_;

  // 1. BFS to find raw frontier cells
  auto frontier_cells = detectFrontierCells(costmap);

  if (frontier_cells.empty()) {
    setOutput("frontiers", std::vector<Frontier>{});
    return BT::NodeStatus::SUCCESS;  // exploration done — caller checks size
  }

  // 2. Cluster frontier cells
  auto clusters = clusterFrontiers(frontier_cells, costmap);

  // 3. Convert clusters → Frontier structs (filtering small clusters)
  std::vector<Frontier> frontiers;
  frontiers.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    if (static_cast<double>(cluster.size()) < min_frontier_size_) {
      continue;
    }
    frontiers.push_back(clusterToFrontier(cluster, costmap));
  }

  setOutput("frontiers", frontiers);
  RCLCPP_DEBUG(node_->get_logger(), "ComputeFrontiers: found %zu frontier clusters.",
    frontiers.size());
  return BT::NodeStatus::SUCCESS;
}

// ─── BFS frontier detection ───────────────────────────────────────────────────

std::vector<std::pair<int, int>> ComputeFrontiers::detectFrontierCells(
  const nav2_msgs::msg::Costmap & costmap)
{
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const auto & data = costmap.data;

  // Cost value meanings for Nav2 costmap:
  //   0         = free
  //   255       = unknown (nav2_costmap_2d::NO_INFORMATION)
  //   1–252     = inflated / lethal
  constexpr uint8_t FREE    = 0;
  constexpr uint8_t UNKNOWN = 255;

  auto idx = [&](int x, int y) { return y * width + x; };
  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };

  std::vector<std::pair<int, int>> frontier_cells;
  std::vector<bool> visited(static_cast<std::size_t>(width * height), false);

  std::queue<std::pair<int, int>> q;

  // Seed BFS from all free cells
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (data[static_cast<std::size_t>(idx(x, y))] == FREE) {
        q.push({x, y});
        visited[static_cast<std::size_t>(idx(x, y))] = true;
      }
    }
  }

  const int dx[] = {-1, 0, 1, 0};
  const int dy[] = {0, -1, 0, 1};

  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();

    // A free cell is a frontier cell if it has at least one unknown neighbour
    bool is_frontier = false;
    for (int d = 0; d < 4; ++d) {
      int nx = cx + dx[d];
      int ny = cy + dy[d];
      if (inBounds(nx, ny) &&
        data[static_cast<std::size_t>(idx(nx, ny))] == UNKNOWN)
      {
        is_frontier = true;
        break;
      }
    }
    if (is_frontier) {
      frontier_cells.emplace_back(cx, cy);
    }
  }

  return frontier_cells;
}

// ─── Clustering (flood-fill on frontier cells) ────────────────────────────────

std::vector<std::vector<std::pair<int, int>>> ComputeFrontiers::clusterFrontiers(
  const std::vector<std::pair<int, int>> & cells,
  const nav2_msgs::msg::Costmap & costmap)
{
  const double res = costmap.metadata.resolution;
  const int cluster_radius =
    std::max(1, static_cast<int>(std::ceil(clustering_distance_ / res)));

  std::unordered_set<int> cell_set;
  const int width = static_cast<int>(costmap.metadata.size_x);
  for (const auto & [x, y] : cells) {
    cell_set.insert(y * width + x);
  }

  std::unordered_set<int> visited_set;
  std::vector<std::vector<std::pair<int, int>>> clusters;

  for (const auto & [sx, sy] : cells) {
    int key = sy * width + sx;
    if (visited_set.count(key)) {
      continue;
    }

    // BFS within clustering distance
    std::vector<std::pair<int, int>> cluster;
    std::queue<std::pair<int, int>> q;
    q.push({sx, sy});
    visited_set.insert(key);

    while (!q.empty()) {
      auto [cx, cy] = q.front();
      q.pop();
      cluster.emplace_back(cx, cy);

      for (int dy = -cluster_radius; dy <= cluster_radius; ++dy) {
        for (int dx = -cluster_radius; dx <= cluster_radius; ++dx) {
          int nx = cx + dx;
          int ny = cy + dy;
          int nkey = ny * width + nx;
          if (cell_set.count(nkey) && !visited_set.count(nkey)) {
            visited_set.insert(nkey);
            q.push({nx, ny});
          }
        }
      }
    }

    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

// ─── Cluster → Frontier ───────────────────────────────────────────────────────

Frontier ComputeFrontiers::clusterToFrontier(
  const std::vector<std::pair<int, int>> & cluster,
  const nav2_msgs::msg::Costmap & costmap)
{
  const double res  = costmap.metadata.resolution;
  const double ox   = costmap.metadata.origin.position.x;
  const double oy   = costmap.metadata.origin.position.y;

  double sum_x = 0.0, sum_y = 0.0;
  for (const auto & [cx, cy] : cluster) {
    sum_x += cx;
    sum_y += cy;
  }
  double mean_cx = sum_x / cluster.size();
  double mean_cy = sum_y / cluster.size();

  Frontier f;
  f.centroid_x = ox + (mean_cx + 0.5) * res;
  f.centroid_y = oy + (mean_cy + 0.5) * res;
  f.size = static_cast<int>(cluster.size());

  f.goal_pose.header.frame_id = costmap.header.frame_id;
  f.goal_pose.header.stamp    = costmap.header.stamp;
  f.goal_pose.pose.position.x = f.centroid_x;
  f.goal_pose.pose.position.y = f.centroid_y;
  f.goal_pose.pose.position.z = 0.0;
  f.goal_pose.pose.orientation.w = 1.0;  // facing "forward" — Nav2 will plan

  return f;
}

}  // namespace hermes_navigate

// ─── Plugin registration ─────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::ComputeFrontiers>("ComputeFrontiers");
}
