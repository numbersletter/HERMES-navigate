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

#include "hermes_navigate/score_frontiers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ScoreFrontiers::ScoreFrontiers(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("score_frontiers_node");

  // Parameter defaults
  alpha_        = 1.0;
  beta_         = 0.5;
  info_radius_m_ = 2.0;

  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      latest_costmap_ = msg;
    });
}

BT::PortsList ScoreFrontiers::providedPorts()
{
  return {
    BT::InputPort<std::vector<Frontier>>("frontiers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::OutputPort<std::vector<ScoredFrontier>>("scored_frontiers"),
  };
}

// ─── Tick ─────────────────────────────────────────────────────────────────────

BT::NodeStatus ScoreFrontiers::tick()
{
  rclcpp::spin_some(node_);

  auto frontiers_res = getInput<std::vector<Frontier>>("frontiers");
  if (!frontiers_res) {
    RCLCPP_WARN(node_->get_logger(), "ScoreFrontiers: no frontiers on blackboard.");
    return BT::NodeStatus::FAILURE;
  }
  const auto & frontiers = frontiers_res.value();

  if (frontiers.empty()) {
    setOutput("scored_frontiers", std::vector<ScoredFrontier>{});
    return BT::NodeStatus::SUCCESS;
  }

  auto pose_res = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");

  if (!latest_costmap_) {
    RCLCPP_WARN(node_->get_logger(), "ScoreFrontiers: no costmap received yet.");
    return BT::NodeStatus::FAILURE;
  }

  const auto & costmap = *latest_costmap_;
  const double res  = costmap.metadata.resolution;
  const double ox   = costmap.metadata.origin.position.x;
  const double oy   = costmap.metadata.origin.position.y;
  const int width   = static_cast<int>(costmap.metadata.size_x);
  const int height  = static_cast<int>(costmap.metadata.size_y);

  // Convert world coords to grid coords
  auto toGrid = [&](double wx, double wy, int & gx, int & gy) -> bool {
    gx = static_cast<int>((wx - ox) / res);
    gy = static_cast<int>((wy - oy) / res);
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
  };

  // Robot position in grid
  int robot_gx = 0, robot_gy = 0;
  bool have_robot_pos = false;
  if (pose_res) {
    const auto & pose = pose_res.value();
    have_robot_pos = toGrid(
      pose.pose.position.x, pose.pose.position.y,
      robot_gx, robot_gy);
  }

  const int info_radius_cells = std::max(1, static_cast<int>(info_radius_m_ / res));

  std::vector<ScoredFrontier> scored;
  scored.reserve(frontiers.size());

  for (const auto & f : frontiers) {
    int fx, fy;
    if (!toGrid(f.centroid_x, f.centroid_y, fx, fy)) {
      continue;
    }

    // Information gain — count unknown cells in radius
    double info_gain = static_cast<double>(countUnknownCells(costmap, fx, fy, info_radius_cells));

    // Traversal cost — A* path length in metres, or fallback to Euclidean
    double traversal_cost = 0.0;
    if (have_robot_pos) {
      double astar = aStarCost(costmap, robot_gx, robot_gy, fx, fy);
      if (astar < 0.0) {
        // No path found — use Euclidean distance as pessimistic estimate
        double dx = f.centroid_x - (pose_res ? pose_res.value().pose.position.x : 0.0);
        double dy = f.centroid_y - (pose_res ? pose_res.value().pose.position.y : 0.0);
        traversal_cost = std::hypot(dx, dy);
      } else {
        traversal_cost = astar;
      }
    }

    ScoredFrontier sf;
    sf.frontier = f;
    sf.score    = alpha_ * info_gain - beta_ * traversal_cost;
    scored.push_back(sf);
  }

  // Sort descending by score
  std::sort(scored.begin(), scored.end(),
    [](const ScoredFrontier & a, const ScoredFrontier & b) {
      return a.score > b.score;
    });

  setOutput("scored_frontiers", scored);
  return BT::NodeStatus::SUCCESS;
}

// ─── Information gain helper ──────────────────────────────────────────────────

int ScoreFrontiers::countUnknownCells(
  const nav2_msgs::msg::Costmap & costmap,
  int cx, int cy, int radius_cells)
{
  constexpr uint8_t UNKNOWN = 255;
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const auto & data = costmap.data;

  int count = 0;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy > radius_cells * radius_cells) {
        continue;
      }
      int nx = cx + dx;
      int ny = cy + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (data[static_cast<std::size_t>(ny * width + nx)] == UNKNOWN) {
        ++count;
      }
    }
  }
  return count;
}

// ─── A* path cost ─────────────────────────────────────────────────────────────

double ScoreFrontiers::aStarCost(
  const nav2_msgs::msg::Costmap & costmap,
  int start_x, int start_y,
  int goal_x,  int goal_y)
{
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const double res = costmap.metadata.resolution;
  const auto & data = costmap.data;

  constexpr uint8_t LETHAL  = 253;
  constexpr uint8_t UNKNOWN = 255;

  if (start_x == goal_x && start_y == goal_y) {
    return 0.0;
  }

  auto idx = [&](int x, int y) { return static_cast<std::size_t>(y * width + x); };
  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };
  auto heuristic = [&](int x, int y) -> double {
    return std::hypot(static_cast<double>(x - goal_x),
      static_cast<double>(y - goal_y)) * res;
  };

  // g-cost map (distance so far)
  std::vector<double> g(static_cast<std::size_t>(width * height),
    std::numeric_limits<double>::infinity());
  g[idx(start_x, start_y)] = 0.0;

  // Priority queue: (f=g+h, x, y)
  using Node = std::tuple<double, int, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  open.emplace(heuristic(start_x, start_y), start_x, start_y);

  const int dx[] = {-1, 0, 1, 0, -1, -1,  1,  1};
  const int dy[] = {0, -1, 0, 1, -1,  1, -1,  1};
  const double step_cost[] = {res, res, res, res,
    res * 1.414, res * 1.414, res * 1.414, res * 1.414};

  while (!open.empty()) {
    auto [f, cx, cy] = open.top();
    open.pop();

    if (cx == goal_x && cy == goal_y) {
      return g[idx(cx, cy)];
    }

    if (f > g[idx(cx, cy)] + heuristic(cx, cy) + 1e-6) {
      continue;  // stale entry
    }

    for (int d = 0; d < 8; ++d) {
      int nx = cx + dx[d];
      int ny = cy + dy[d];
      if (!inBounds(nx, ny)) {
        continue;
      }
      uint8_t cost_val = data[idx(nx, ny)];
      if (cost_val >= LETHAL || cost_val == UNKNOWN) {
        continue;  // obstacle or unknown — skip
      }
      double ng = g[idx(cx, cy)] + step_cost[d];
      if (ng < g[idx(nx, ny)]) {
        g[idx(nx, ny)] = ng;
        open.emplace(ng + heuristic(nx, ny), nx, ny);
      }
    }
  }

  return -1.0;  // no path found
}

}  // namespace hermes_navigate

// ─── Plugin registration ──────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::ScoreFrontiers>("ScoreFrontiers");
}
