// Copyright 2026 Rodriguez Sage
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

#include "tb4_astar_planner/astar_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace tb4_astar_planner
{
namespace
{

constexpr unsigned char kNoInformation = 255;
constexpr double kDiagonalDistance = 1.4142135623730951;

struct QueueEntry
{
  unsigned int cell;
  double priority;
};

struct LowestPriority
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const
  {
    return left.priority > right.priority;
  }
};

double heuristic(
  unsigned int cell,
  unsigned int goal,
  unsigned int width,
  bool use_diagonal)
{
  const auto x = static_cast<int>(cell % width);
  const auto y = static_cast<int>(cell / width);
  const auto goal_x = static_cast<int>(goal % width);
  const auto goal_y = static_cast<int>(goal / width);
  const auto dx = std::abs(goal_x - x);
  const auto dy = std::abs(goal_y - y);

  if (!use_diagonal) {
    return static_cast<double>(dx + dy);
  }

  const auto diagonal = std::min(dx, dy);
  const auto straight = std::max(dx, dy) - diagonal;
  return kDiagonalDistance * diagonal + straight;
}

bool isTraversable(
  unsigned int cell,
  const unsigned char * costs,
  const SearchOptions & options)
{
  const auto cost = costs[cell];
  if (cost == kNoInformation) {
    return options.allow_unknown;
  }
  return cost < options.lethal_cost;
}

double traversalCost(
  unsigned int cell,
  const unsigned char * costs,
  double distance,
  double cost_penalty)
{
  const auto cost = costs[cell];
  const double normalized =
    cost == kNoInformation ? 1.0 :
    static_cast<double>(std::min<unsigned int>(cost, 252U)) / 252.0;
  return distance * (1.0 + cost_penalty * normalized);
}

}  // namespace

SearchResult AStarSearch::findPath(
  unsigned int start,
  unsigned int goal,
  const unsigned char * costs,
  unsigned int width,
  unsigned int height,
  const SearchOptions & options) const
{
  if (costs == nullptr || width == 0U || height == 0U) {
    throw std::invalid_argument("Costmap must be non-empty");
  }
  if (options.cost_penalty < 0.0) {
    throw std::invalid_argument("Cost penalty must be non-negative");
  }

  const auto cell_count = static_cast<std::size_t>(width) * height;
  if (start >= cell_count || goal >= cell_count) {
    throw std::out_of_range("Start or goal is outside the costmap");
  }

  SearchResult result;
  if (start == goal) {
    result.success = true;
    result.cells.push_back(start);
    return result;
  }
  if (!isTraversable(goal, costs, options)) {
    return result;
  }

  const auto infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(cell_count, infinity);
  std::vector<unsigned int> parent(
    cell_count, std::numeric_limits<unsigned int>::max());
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LowestPriority> open;

  g_score[start] = 0.0;
  open.push({start, heuristic(start, goal, width, options.use_diagonal)});

  constexpr std::array<std::pair<int, int>, 8> neighbors = {{
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  }};

  while (!open.empty()) {
    const auto current = open.top().cell;
    open.pop();

    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    ++result.expanded_nodes;

    if (current == goal) {
      result.success = true;
      result.path_cost = g_score[goal];
      break;
    }

    const auto current_x = static_cast<int>(current % width);
    const auto current_y = static_cast<int>(current / width);
    for (const auto & [dx, dy] : neighbors) {
      const bool diagonal = dx != 0 && dy != 0;
      if (diagonal && !options.use_diagonal) {
        continue;
      }

      const auto next_x = current_x + dx;
      const auto next_y = current_y + dy;
      if (
        next_x < 0 || next_y < 0 ||
        next_x >= static_cast<int>(width) ||
        next_y >= static_cast<int>(height))
      {
        continue;
      }

      const auto next =
        static_cast<unsigned int>(next_y) * width +
        static_cast<unsigned int>(next_x);
      if (closed[next] || !isTraversable(next, costs, options)) {
        continue;
      }

      if (diagonal) {
        const auto side_x =
          static_cast<unsigned int>(current_y) * width +
          static_cast<unsigned int>(next_x);
        const auto side_y =
          static_cast<unsigned int>(next_y) * width +
          static_cast<unsigned int>(current_x);
        if (
          !isTraversable(side_x, costs, options) ||
          !isTraversable(side_y, costs, options))
        {
          continue;
        }
      }

      const double distance = diagonal ? kDiagonalDistance : 1.0;
      const double tentative =
        g_score[current] +
        traversalCost(next, costs, distance, options.cost_penalty);
      if (tentative >= g_score[next]) {
        continue;
      }

      parent[next] = current;
      g_score[next] = tentative;
      open.push(
        {
          next,
          tentative + heuristic(next, goal, width, options.use_diagonal)
        });
    }
  }

  if (!result.success) {
    return result;
  }

  for (auto cell = goal; cell != start; cell = parent[cell]) {
    result.cells.push_back(cell);
    if (parent[cell] == std::numeric_limits<unsigned int>::max()) {
      return SearchResult{};
    }
  }
  result.cells.push_back(start);
  std::reverse(result.cells.begin(), result.cells.end());
  return result;
}

}  // namespace tb4_astar_planner
