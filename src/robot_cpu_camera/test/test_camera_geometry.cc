// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#include <limits>
#include <gtest/gtest.h>
#include "robot_cpu_camera/CameraGeometry.hh"

TEST(CameraGeometry, IndependentFovsAndPixelCenters)
{
  constexpr double rad = 3.14159265358979323846 / 180.0;
  const auto rays = robot_cpu_camera::CameraRays(161, 91, 87 * rad, 58 * rad);
  EXPECT_EQ(rays.size(), 161u * 91u);
  EXPECT_EQ(rays[45 * 161 + 80], gz::math::Vector3d(1, 0, 0));
  EXPECT_NEAR(rays.front().Y(), std::tan(43.5 * rad) * 160 / 161, 1e-12);
  EXPECT_NEAR(rays.front().Z(), std::tan(29 * rad) * 90 / 91, 1e-12);
  EXPECT_NEAR(rays.back().Y(), -rays.front().Y(), 1e-12);
  EXPECT_NEAR(rays.back().Z(), -rays.front().Z(), 1e-12);
}

TEST(CameraGeometry, ClipsOpticalDepthNotEuclideanRange)
{
  EXPECT_TRUE(robot_cpu_camera::ValidDepth({2.9, 2, 1}, 0.3, 3));
  EXPECT_FALSE(robot_cpu_camera::ValidDepth({0.2, 1, 1}, 0.3, 3));
  EXPECT_FALSE(robot_cpu_camera::ValidDepth({3.1, 0, 0}, 0.3, 3));
  EXPECT_FALSE(robot_cpu_camera::ValidDepth(
      {1, std::numeric_limits<double>::quiet_NaN(), 0}, 0.3, 3));
}

TEST(CameraGeometry, RejectsInvalidGridAndFov)
{
  EXPECT_THROW(robot_cpu_camera::CameraRays(0, 90, 1, 1), std::invalid_argument);
  EXPECT_THROW(robot_cpu_camera::CameraRays(160, 90, 4, 1), std::invalid_argument);
  EXPECT_THROW(robot_cpu_camera::CameraRays(160, 90, 1, 0), std::invalid_argument);
}
