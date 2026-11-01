// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include "robot_mppi_critics/forward_travel.hpp"

TEST(ForwardTravel, PrefersForwardButAllowsSmallTrackingCorrections)
{
  EXPECT_FLOAT_EQ(robot_mppi_critics::lateralExcess(0.6f, 0.0f, 0.268f), 0.0f);
  EXPECT_FLOAT_EQ(robot_mppi_critics::lateralExcess(0.6f, 0.1f, 0.268f), 0.0f);
  EXPECT_GT(robot_mppi_critics::lateralExcess(0.0f, 0.3f, 0.268f), 0.29f);
  EXPECT_GT(robot_mppi_critics::lateralExcess(0.3f, 0.3f, 0.268f), 0.2f);
}

TEST(ForwardTravel, OnlyKnownForwardObstaclesRelaxPreference)
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.05, -2.5, -2.5, 0);
  const auto blocked = [&](double yaw) {
      return robot_mppi_critics::obstacleAhead(map, 0, 0, yaw, 0.4, 0.32, 1.5);
    };
  EXPECT_FALSE(blocked(0));
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(1.0, 0, x, y));
  map.setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(blocked(0));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(blocked(0));
  EXPECT_FALSE(blocked(1.57079632679));
  map.setCost(x, y, 0);
  ASSERT_TRUE(map.worldToMap(0, 1.0, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(blocked(1.57079632679));
  EXPECT_FALSE(blocked(0));
}
