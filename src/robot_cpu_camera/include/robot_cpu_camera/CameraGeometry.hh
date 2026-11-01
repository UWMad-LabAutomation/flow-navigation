// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#ifndef ROBOT_CPU_CAMERA_CAMERA_GEOMETRY_HH_
#define ROBOT_CPU_CAMERA_CAMERA_GEOMETRY_HH_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <gz/math/Vector3.hh>

namespace robot_cpu_camera
{
// Rays pass through coarse pixel centers using independent pinhole focal
// lengths. X is optical depth; Y is left and Z is up (camera_depth_frame).
inline std::vector<gz::math::Vector3d> CameraRays(
  unsigned int width, unsigned int height, double hfov, double vfov)
{
  constexpr double pi = 3.14159265358979323846;
  if (!width || !height || static_cast<uint64_t>(width) * height > 1048576 ||
    !std::isfinite(hfov) || !std::isfinite(vfov) ||
    hfov <= 0 || hfov >= pi || vfov <= 0 || vfov >= pi)
  {
    throw std::invalid_argument("Invalid CPU camera ray grid or FOV");
  }
  const double fx = width / (2.0 * std::tan(hfov / 2));
  const double fy = height / (2.0 * std::tan(vfov / 2));
  std::vector<gz::math::Vector3d> rays;
  rays.reserve(static_cast<size_t>(width) * height);
  for (unsigned int row = 0; row < height; ++row)
    for (unsigned int col = 0; col < width; ++col)
      rays.emplace_back(1.0, (width / 2.0 - col - 0.5) / fx,
        (height / 2.0 - row - 0.5) / fy);
  return rays;
}

inline bool ValidDepth(const gz::math::Vector3d &point, double near, double far)
{
  return point.IsFinite() && point.X() >= near && point.X() <= far;
}
}  // namespace robot_cpu_camera
#endif  // ROBOT_CPU_CAMERA_CAMERA_GEOMETRY_HH_
