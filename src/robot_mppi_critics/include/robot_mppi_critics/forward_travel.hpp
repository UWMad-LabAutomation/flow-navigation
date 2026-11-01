// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#ifndef ROBOT_MPPI_CRITICS__FORWARD_TRAVEL_HPP_
#define ROBOT_MPPI_CRITICS__FORWARD_TRAVEL_HPP_

#include <algorithm>
#include <cmath>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_mppi_critics
{
inline float lateralExcess(float vx, float vy, float tangent)
{
  return std::max(0.0f, std::fabs(vy) - tangent * std::max(vx, 0.0f));
}

inline bool obstacleAhead(
  const nav2_costmap_2d::Costmap2D & map, double x, double y, double yaw,
  double half_length, double half_width, double lookahead)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double step = map.getResolution() * 0.5;
  for (double forward = half_length; forward <= lookahead; forward += step) {
    for (double lateral = -half_width; lateral <= half_width; lateral += step) {
      unsigned int mx, my;
      if (map.worldToMap(x + c * forward - s * lateral,
                        y + s * forward + c * lateral, mx, my) &&
        map.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace robot_mppi_critics
#endif  // ROBOT_MPPI_CRITICS__FORWARD_TRAVEL_HPP_
