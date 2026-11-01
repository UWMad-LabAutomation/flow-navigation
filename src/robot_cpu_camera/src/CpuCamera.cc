// Copyright 2026 Zach Gaier
// Licensed under the Apache License, Version 2.0.
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "robot_cpu_camera/CameraGeometry.hh"

namespace robot_cpu_camera
{
class CpuCamera : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &model,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    if (!ecm.Component<gz::sim::components::Model>(model))
      throw std::runtime_error("CpuCamera must be attached to a model");
    const auto width = sdf->Get<unsigned int>("width");
    const auto height = sdf->Get<unsigned int>("height");
    this->near = sdf->Get<double>("near");
    this->far = sdf->Get<double>("far");
    const double rate = sdf->Get<double>("update_rate");
    if (!std::isfinite(this->near) || !std::isfinite(this->far) ||
      this->near <= 0 || this->far <= this->near || !std::isfinite(rate) || rate <= 0)
      throw std::invalid_argument("Invalid CPU camera depth limits or rate");
    this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
    if (this->period.count() <= 0)
      throw std::invalid_argument("CPU camera rate is too large");
    for (const auto &direction : CameraRays(width, height,
      sdf->Get<double>("hfov"), sdf->Get<double>("vfov")))
    {
      // Cast from the optical origin, then reject too-near first hits. Starting
      // at the near plane would incorrectly see through nearby occluders.
      this->rays.push_back({gz::math::Vector3d::Zero, direction * this->far});
    }

    // Harmonic's DART ray queries require its Bullet collision detector.
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    ecm.SetComponentData<gz::sim::components::PhysicsCollisionDetector>(world, "bullet");
    this->entity = ecm.CreateEntity();
    ecm.SetParentEntity(this->entity, model);
    ecm.CreateComponent(this->entity, gz::sim::components::ParentEntity(model));
    ecm.CreateComponent(this->entity, gz::sim::components::Name("cpu_depth_rays"));
    ecm.CreateComponent(this->entity,
      gz::sim::components::Pose(sdf->Get<gz::math::Pose3d>("pose")));
    ecm.CreateComponent(this->entity, gz::sim::components::RaycastData());

    gz::msgs::InitPointCloudPacked(this->cloud, "camera_depth_frame", false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
    this->cloud.set_width(width);
    this->cloud.set_height(height);
    this->cloud.set_row_step(width * this->cloud.point_step());
    this->cloud.set_is_dense(false);
    const uint16_t endian = 1;
    this->cloud.set_is_bigendian(*reinterpret_cast<const char *>(&endian) == 0);
    this->cloud.mutable_data()->resize(height * this->cloud.row_step());
    this->publisher = this->node.Advertise<gz::msgs::PointCloudPacked>(
      sdf->Get<std::string>("topic"));
    if (!this->publisher)
      throw std::runtime_error("Cannot advertise CPU camera point cloud");
    gzmsg << "FLOW CPU camera: " << width << "x" << height << " rays at "
          << rate << " Hz; physics collision geometry, no sensor rendering.\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    auto *data = ecm.Component<gz::sim::components::RaycastData>(this->entity);
    if (!data)
      return;
    if (info.dt < std::chrono::steady_clock::duration::zero())
      this->next = info.simTime;
    this->capture = !info.paused && info.simTime >= this->next;
    if (this->capture)
    {
      data->Data().rays = this->rays;
      // Preserve the requested average rate when the physics step does not
      // divide the camera period (e.g. 1 ms physics and 30 Hz camera).
      const auto skipped = (info.simTime - this->next) / this->period;
      this->next += (skipped + 1) * this->period;
    }
    else
    {
      // Physics runs at 1000 Hz. Do not ray-cast on steps between camera frames.
      data->Data().rays.clear();
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override
  {
    if (!this->capture)
      return;
    const auto *data = ecm.Component<gz::sim::components::RaycastData>(this->entity);
    if (!data || data->Data().results.size() != this->rays.size())
    {
      if (!this->warned)
        gzerr << "CPU camera received no physics ray results. Check DART/Bullet support.\n";
      this->warned = true;
      return;
    }
    auto *bytes = this->cloud.mutable_data()->data();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < this->rays.size(); ++i)
    {
      const auto &hit = data->Data().results[i];
      float xyz[3] = {nan, nan, nan};
      if (std::isfinite(hit.fraction) && hit.fraction >= 0 && hit.fraction <= 1 &&
        ValidDepth(hit.point, this->near, this->far))
      {
        xyz[0] = static_cast<float>(hit.point.X());
        xyz[1] = static_cast<float>(hit.point.Y());
        xyz[2] = static_cast<float>(hit.point.Z());
      }
      std::memcpy(bytes + i * this->cloud.point_step(), xyz, sizeof(xyz));
    }
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec);
    auto *stamp = this->cloud.mutable_header()->mutable_stamp();
    stamp->set_sec(sec.count());
    stamp->set_nsec(nsec.count());
    this->publisher.Publish(this->cloud);
  }

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  gz::msgs::PointCloudPacked cloud;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  std::vector<gz::sim::components::RayInfo> rays;
  std::chrono::steady_clock::duration period{}, next{};
  double near{}, far{};
  bool capture{false}, warned{false};
};
}  // namespace robot_cpu_camera

GZ_ADD_PLUGIN(robot_cpu_camera::CpuCamera, gz::sim::System,
  robot_cpu_camera::CpuCamera::ISystemConfigure,
  robot_cpu_camera::CpuCamera::ISystemPreUpdate,
  robot_cpu_camera::CpuCamera::ISystemPostUpdate)
