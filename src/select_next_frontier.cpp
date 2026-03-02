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

#include "hermes_navigate/select_next_frontier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

SelectNextFrontier::SelectNextFrontier(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  hysteresis_factor_  = 0.15;  // 15 % score improvement required to switch
  min_frontier_score_ = 0.0;
}

BT::PortsList SelectNextFrontier::providedPorts()
{
  return {
    BT::InputPort<std::vector<ScoredFrontier>>("scored_frontiers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::OutputPort<bool>("exploration_done"),
  };
}

// ─── Tick ─────────────────────────────────────────────────────────────────────

BT::NodeStatus SelectNextFrontier::tick()
{
  auto scored_res = getInput<std::vector<ScoredFrontier>>("scored_frontiers");
  if (!scored_res) {
    setOutput("exploration_done", true);
    return BT::NodeStatus::FAILURE;
  }

  const auto & scored = scored_res.value();

  // Filter viable frontiers
  std::vector<const ScoredFrontier *> viable;
  for (const auto & sf : scored) {
    if (sf.score >= min_frontier_score_) {
      viable.push_back(&sf);
    }
  }

  if (viable.empty()) {
    setOutput("exploration_done", true);
    has_active_goal_ = false;
    return BT::NodeStatus::SUCCESS;  // exploration done
  }

  setOutput("exploration_done", false);

  // Greedy nearest-neighbour ordering from robot's current position
  auto pose_res = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");

  // Start position — prefer robot pose; fallback to first viable frontier
  geometry_msgs::msg::PoseStamped current_pos;
  if (pose_res) {
    current_pos = pose_res.value();
  } else if (!viable.empty()) {
    current_pos = viable[0]->frontier.goal_pose;
  }

  // Build greedy ordered list
  std::vector<bool> used(viable.size(), false);
  std::vector<const ScoredFrontier *> ordered;
  ordered.reserve(viable.size());

  geometry_msgs::msg::PoseStamped ref = current_pos;
  for (std::size_t i = 0; i < viable.size(); ++i) {
    double best_dist = std::numeric_limits<double>::max();
    int best_idx = -1;
    for (std::size_t j = 0; j < viable.size(); ++j) {
      if (!used[j]) {
        double d = dist(ref, viable[j]->frontier.goal_pose);
        if (d < best_dist) {
          best_dist = d;
          best_idx  = static_cast<int>(j);
        }
      }
    }
    if (best_idx < 0) {
      break;
    }
    used[static_cast<std::size_t>(best_idx)] = true;
    ordered.push_back(viable[static_cast<std::size_t>(best_idx)]);
    ref = ordered.back()->frontier.goal_pose;
  }

  // Candidate — best scored (ordered list is already from robot → nearest
  // next, but score is the primary ranking criterion)
  // Re-rank by score within the ordered sequence and pick the highest.
  const ScoredFrontier * best = nullptr;
  for (const auto * sf : ordered) {
    if (!best || sf->score > best->score) {
      best = sf;
    }
  }

  if (!best) {
    setOutput("exploration_done", true);
    return BT::NodeStatus::FAILURE;
  }

  // Hysteresis: only switch if the new goal is significantly better
  if (has_active_goal_) {
    double required_score = active_goal_score_ * (1.0 + hysteresis_factor_);
    if (best->score < required_score) {
      // Keep navigating to active goal
      setOutput("goal", active_goal_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Accept new goal
  has_active_goal_    = true;
  active_goal_        = best->frontier.goal_pose;
  active_goal_score_  = best->score;

  setOutput("goal", active_goal_);
  return BT::NodeStatus::SUCCESS;
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

double SelectNextFrontier::dist(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  double dx = a.pose.position.x - b.pose.position.x;
  double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

}  // namespace hermes_navigate

// ─── Plugin registration ──────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::SelectNextFrontier>("SelectNextFrontier");
}
