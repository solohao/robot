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

#ifndef TB4_ASTAR_PLANNER__ASTAR_SEARCH_HPP_
#define TB4_ASTAR_PLANNER__ASTAR_SEARCH_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tb4_astar_planner
{

struct SearchOptions
{
  unsigned char lethal_cost{253};
  bool allow_unknown{false};
  bool use_diagonal{true};
  double cost_penalty{2.0};
};

struct SearchResult
{
  bool success{false};
  std::vector<unsigned int> cells;
  double path_cost{0.0};
  std::size_t expanded_nodes{0};
};

class AStarSearch
{
public:
  SearchResult findPath(
    unsigned int start,
    unsigned int goal,
    const unsigned char * costs,
    unsigned int width,
    unsigned int height,
    const SearchOptions & options) const;
};

}  // namespace tb4_astar_planner

#endif  // TB4_ASTAR_PLANNER__ASTAR_SEARCH_HPP_
