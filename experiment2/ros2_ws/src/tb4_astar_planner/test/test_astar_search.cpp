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

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "tb4_astar_planner/astar_search.hpp"

namespace tb4_astar_planner
{
namespace
{

constexpr unsigned int kWidth = 10;
constexpr unsigned int kHeight = 10;
constexpr unsigned char kLethal = 254;

TEST(AStarSearch, FindsDiagonalPathAcrossEmptyMap)
{
  std::vector<unsigned char> costs(kWidth * kHeight, 0);
  const auto result = AStarSearch().findPath(
    0, kWidth * kHeight - 1, costs.data(), kWidth, kHeight, SearchOptions{});

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.cells.front(), 0U);
  ASSERT_EQ(result.cells.back(), kWidth * kHeight - 1);
  EXPECT_EQ(result.cells.size(), 10U);
}

TEST(AStarSearch, RoutesThroughWallOpening)
{
  std::vector<unsigned char> costs(kWidth * kHeight, 0);
  for (unsigned int y = 0; y < kHeight; ++y) {
    costs[y * kWidth + 5] = kLethal;
  }
  costs[7 * kWidth + 5] = 0;

  const auto result = AStarSearch().findPath(
    2 * kWidth + 2,
    2 * kWidth + 8,
    costs.data(),
    kWidth,
    kHeight,
    SearchOptions{});

  ASSERT_TRUE(result.success);
  EXPECT_NE(
    std::find(result.cells.begin(), result.cells.end(), 7 * kWidth + 5),
    result.cells.end());
  for (const auto cell : result.cells) {
    EXPECT_LT(costs[cell], 253);
  }
}

TEST(AStarSearch, ReportsNoPathForSolidWall)
{
  std::vector<unsigned char> costs(kWidth * kHeight, 0);
  for (unsigned int y = 0; y < kHeight; ++y) {
    costs[y * kWidth + 5] = kLethal;
  }

  const auto result = AStarSearch().findPath(
    2 * kWidth + 2,
    2 * kWidth + 8,
    costs.data(),
    kWidth,
    kHeight,
    SearchOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.cells.empty());
}

TEST(AStarSearch, DoesNotCutBlockedCorners)
{
  std::vector<unsigned char> costs(9, 0);
  costs[1] = kLethal;
  costs[3] = kLethal;

  const auto result = AStarSearch().findPath(
    0, 4, costs.data(), 3, 3, SearchOptions{});

  EXPECT_FALSE(result.success);
}

TEST(AStarSearch, AvoidsHighCostCells)
{
  std::vector<unsigned char> costs(7 * 5, 0);
  costs[2 * 7 + 2] = 220;
  costs[2 * 7 + 3] = 220;
  costs[2 * 7 + 4] = 220;

  SearchOptions options;
  options.cost_penalty = 8.0;
  const auto result = AStarSearch().findPath(
    2 * 7,
    2 * 7 + 6,
    costs.data(),
    7,
    5,
    options);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(
    std::count_if(
      result.cells.begin(), result.cells.end(),
      [&costs](unsigned int cell) {return costs[cell] > 0;}),
    0);
}

}  // namespace
}  // namespace tb4_astar_planner
