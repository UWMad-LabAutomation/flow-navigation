// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <stdexcept>
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_mppi_critics/forward_travel.hpp"

namespace mppi::critics
{
class ForwardTravelCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto get = parameters_handler_->getParamGetter(name_);
    get(weight_, "cost_weight", 60.0f);
    get(avoidance_weight_, "avoidance_weight", 4.0f);
    get(half_angle_, "forward_half_angle", 0.261799f);
    get(lookahead_, "obstacle_lookahead", 1.5f);
    get(alignment_, "alignment_tolerance", 0.5f);
    get(half_length_, "robot_half_length", 0.3955f);
    get(half_width_, "robot_half_width", 0.321f);
    if (!std::isfinite(weight_) || !std::isfinite(avoidance_weight_) ||
      weight_ <= 0 || avoidance_weight_ < 0 || avoidance_weight_ > weight_ ||
      !std::isfinite(half_angle_) || half_angle_ <= 0 || half_angle_ >= 1.570796f ||
      !std::isfinite(lookahead_) || !std::isfinite(half_length_) ||
      half_length_ <= 0 || lookahead_ <= half_length_ ||
      !std::isfinite(half_width_) || half_width_ <= 0 ||
      !std::isfinite(alignment_) || alignment_ <= 0 || alignment_ > 1.570796f)
    {
      throw std::invalid_argument("Invalid forward travel critic parameters");
    }
  }

  void score(CriticData & data) override
  {
    if (!enabled_ || !data.motion_model->isHolonomic()) {return;}
    const auto & pose = data.state.pose.pose;
    const auto & q = pose.orientation;
    const double heading = std::atan2(2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z));
    bool aligned = false;
    if (data.path.x.size() > 1) {
      size_t index = data.path.x.size() - 1;
      for (size_t i = 1; i < data.path.x.size(); ++i) {
        if (std::hypot(data.path.x(i) - pose.position.x,
            data.path.y(i) - pose.position.y) >= 0.6)
        {
          index = i;
          break;
        }
      }
      const double bearing = std::atan2(data.path.y(index) - pose.position.y,
          data.path.x(index) - pose.position.x);
      aligned = std::fabs(std::atan2(std::sin(bearing - heading),
          std::cos(bearing - heading))) < alignment_;
    }
    const bool avoiding = aligned && robot_mppi_critics::obstacleAhead(
      *costmap_, pose.position.x, pose.position.y, heading,
      half_length_, half_width_, lookahead_);
    const float weight = avoiding ? avoidance_weight_ : weight_;
    const float tangent = std::tan(half_angle_);
    for (size_t batch = 0; batch < data.state.vx.shape(0); ++batch) {
      float penalty = 0.0f;
      for (size_t step = 0; step < data.state.vx.shape(1); ++step) {
        penalty += robot_mppi_critics::lateralExcess(
          data.state.vx(batch, step), data.state.vy(batch, step), tangent);
      }
      data.costs(batch) += weight * data.model_dt * penalty;
    }
  }

private:
  float weight_, avoidance_weight_, half_angle_, lookahead_, alignment_;
  float half_length_, half_width_;
};
}  // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::ForwardTravelCritic, mppi::critics::CriticFunction)
