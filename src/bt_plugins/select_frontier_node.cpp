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

#include "hermes_navigate/bt_plugins/select_frontier_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

SelectFrontierNode::SelectFrontierNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::SyncActionNode(name, config),
  parent_(parent)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("SelectFrontierNode: parent node expired.");
  }

  node->declare_parameter("select_frontier.hysteresis_factor",
    rclcpp::ParameterValue(0.15));
  node->declare_parameter("select_frontier.min_frontier_score",
    rclcpp::ParameterValue(0.0));
  node->declare_parameter("select_frontier.blacklist_radius_m",
    rclcpp::ParameterValue(0.5));
  hysteresis_factor_  = node->get_parameter("select_frontier.hysteresis_factor").as_double();
  min_frontier_score_ = node->get_parameter("select_frontier.min_frontier_score").as_double();
  blacklist_radius_m_ = node->get_parameter("select_frontier.blacklist_radius_m").as_double();
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList SelectFrontierNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<ScoredFrontier>>("scored_frontiers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("blacklisted_goals"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal"),
    BT::OutputPort<bool>("exploration_done"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus SelectFrontierNode::tick()
{
  auto node = parent_.lock();

  auto scored_res = getInput<std::vector<ScoredFrontier>>("scored_frontiers");
  if (!scored_res) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "SelectFrontierNode: no scored_frontiers port value — setting exploration_done=true.");
    }
    setOutput("exploration_done", true);
    return BT::NodeStatus::FAILURE;
  }

  const auto & scored = scored_res.value();

  // Read the blacklist (may be absent if no frontier has been blacklisted yet).
  std::vector<geometry_msgs::msg::PoseStamped> blacklist;
  auto bl_res =
    getInput<std::vector<geometry_msgs::msg::PoseStamped>>("blacklisted_goals");
  if (bl_res) {
    blacklist = bl_res.value();
  }

  // Helper: returns true if the given goal pose is too close to any blacklisted
  // position and should therefore be excluded from selection.
  // Uses squared-distance comparison to avoid sqrt in the inner loop.
  // Note: this check is O(blacklist_size) per frontier; in typical exploration
  // runs the blacklist remains small (single-digit entries), so this is not a
  // performance concern in practice.
  const double radius_sq = blacklist_radius_m_ * blacklist_radius_m_;
  auto is_blacklisted = [&](const geometry_msgs::msg::PoseStamped & pose) -> bool {
    for (const auto & bl : blacklist) {
      const double dx = pose.pose.position.x - bl.pose.position.x;
      const double dy = pose.pose.position.y - bl.pose.position.y;
      if ((dx * dx + dy * dy) < radius_sq) {
        return true;
      }
    }
    return false;
  };

  // If the currently tracked goal was blacklisted, reset hysteresis so a fresh
  // frontier can be selected on this tick.
  if (has_active_goal_ && is_blacklisted(active_goal_)) {
    if (node) {
      RCLCPP_INFO(node->get_logger(),
        "SelectFrontierNode: active goal (%.2f, %.2f) is now blacklisted — resetting hysteresis.",
        active_goal_.pose.position.x, active_goal_.pose.position.y);
    }
    has_active_goal_ = false;
  }

  // If the currently tracked goal is no longer reported as a frontier (i.e. the
  // robot reached it and the wavefront detector has since mapped the area and
  // removed it from the frontier list), clear hysteresis so the next best
  // frontier is chosen without bias from the stale active goal.  Without this
  // check the hysteresis guard can keep nav_goal pointing at an already-visited
  // position, causing the robot to revisit it indefinitely.
  if (has_active_goal_) {
    bool active_still_frontier = false;
    for (const auto & sf : scored) {
      const double dx =
        active_goal_.pose.position.x - sf.frontier.goal_pose.pose.position.x;
      const double dy =
        active_goal_.pose.position.y - sf.frontier.goal_pose.pose.position.y;
      if ((dx * dx + dy * dy) < radius_sq) {
        active_still_frontier = true;
        break;
      }
    }
    if (!active_still_frontier) {
      if (node) {
        RCLCPP_INFO(node->get_logger(),
          "SelectFrontierNode: active goal (%.2f, %.2f) is no longer a frontier "
          "(reached and mapped) — resetting hysteresis.",
          active_goal_.pose.position.x, active_goal_.pose.position.y);
      }
      has_active_goal_ = false;
    }
  }

  // Find the best viable (above-threshold, non-blacklisted) frontier.
  const ScoredFrontier * best = nullptr;
  std::size_t n_below_threshold = 0;
  std::size_t n_blacklisted = 0;
  for (const auto & sf : scored) {
    if (sf.score < min_frontier_score_) {
      ++n_below_threshold;
    } else if (is_blacklisted(sf.frontier.goal_pose)) {
      ++n_blacklisted;
    } else if (!best || sf.score > best->score) {
      best = &sf;
    }
  }

  if (!best) {
    // If there are frontiers that are above the score threshold but only
    // excluded because they were previously blacklisted, fall back to
    // selecting the best of those rather than halting exploration.  Only the
    // blacklist entries near the chosen frontier are removed so that other
    // genuinely unreachable positions remain excluded.
    if (n_blacklisted > 0) {
      const ScoredFrontier * fallback = nullptr;
      for (const auto & sf : scored) {
        if (sf.score >= min_frontier_score_ && is_blacklisted(sf.frontier.goal_pose)) {
          if (!fallback || sf.score > fallback->score) {
            fallback = &sf;
          }
        }
      }
      if (fallback) {
        if (node) {
          RCLCPP_WARN(node->get_logger(),
            "SelectFrontierNode: all frontiers were blacklisted "
            "(total=%zu, blacklisted=%zu) — removing blacklist entry for "
            "frontier at (%.2f, %.2f) [score=%.1f] and retrying.",
            scored.size(), n_blacklisted,
            fallback->frontier.goal_pose.pose.position.x,
            fallback->frontier.goal_pose.pose.position.y,
            fallback->score);
        }
        // Remove only the blacklist entries near the selected frontier so
        // other genuinely unreachable positions remain excluded.
        const auto & target = fallback->frontier.goal_pose;
        blacklist.erase(
          std::remove_if(
            blacklist.begin(), blacklist.end(),
            [&](const geometry_msgs::msg::PoseStamped & bl) {
              const double dx = target.pose.position.x - bl.pose.position.x;
              const double dy = target.pose.position.y - bl.pose.position.y;
              return (dx * dx + dy * dy) < radius_sq;
            }),
          blacklist.end());
        setOutput("blacklisted_goals", blacklist);
        best = fallback;
        has_active_goal_ = false;
      }
    }

    if (!best) {
      if (node) {
        RCLCPP_WARN(node->get_logger(),
          "SelectFrontierNode: no viable frontier found "
          "(total=%zu, below_threshold=%zu, blacklisted=%zu) — setting exploration_done=true.",
          scored.size(), n_below_threshold, n_blacklisted);
      }
      setOutput("exploration_done", true);
      has_active_goal_ = false;
      return BT::NodeStatus::FAILURE;  // no viable frontier — signal exploration complete
    }
  }

  setOutput("exploration_done", false);

  // Hysteresis: only switch if the new goal is significantly better
  if (has_active_goal_) {
    double required_score = active_goal_score_ * (1.0 + hysteresis_factor_);
    if (best->score < required_score) {
      if (node) {
        RCLCPP_DEBUG(node->get_logger(),
          "SelectFrontierNode: hysteresis — keeping current goal (%.2f, %.2f) "
          "[score=%.1f, best_score=%.1f, required=%.1f].",
          active_goal_.pose.position.x, active_goal_.pose.position.y,
          active_goal_score_, best->score, required_score);
      }
      setOutput("nav_goal", active_goal_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Accept new goal
  const bool goal_changed =
    !has_active_goal_ ||
    active_goal_.pose.position.x != best->frontier.goal_pose.pose.position.x ||
    active_goal_.pose.position.y != best->frontier.goal_pose.pose.position.y;

  has_active_goal_   = true;
  active_goal_       = best->frontier.goal_pose;
  active_goal_score_ = best->score;

  if (node && goal_changed) {
    RCLCPP_INFO(node->get_logger(),
      "SelectFrontierNode: new nav_goal selected → (%.2f, %.2f) [score=%.1f, "
      "total_frontiers=%zu, blacklisted=%zu].",
      active_goal_.pose.position.x, active_goal_.pose.position.y,
      active_goal_score_, scored.size(), n_blacklisted);
  }

  setOutput("nav_goal", active_goal_);
  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void SelectFrontierNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<SelectFrontierNode>(
    "SelectFrontier",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SelectFrontierNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
