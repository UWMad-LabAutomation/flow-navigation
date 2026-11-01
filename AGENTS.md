# Robot Navigation Project Context

## Change policy
- Do not modify files unless the user explicitly asks for changes.
- Default to explaining proposed changes first.
- Before editing, summarize which files will be changed and why.
- Do not delete, rename, or move files without explicit approval.
- Do not run destructive commands without explicit approval.

## Environment
- Ubuntu 24.04 in WSL2
- ROS 2 Jazzy
- Workspace: ~/robot_ws
- Nav2
- Gazebo Harmonic
- RViz2
- Python 3.12
- Packages use Apache-2.0
- Vendored third-party pointcloud_to_laserscan 2.0.2 retains its upstream BSD license.

## Robot
- I2RT LinearBot
- FLOW holonomic mobile base
- Powered caster design
- Fully holonomic
- Base communication: CAN
- Max base speed: 0.63 m/s
- Full LinearBot dimensions: 0.542 x 0.751 x 1.703 m
- Vertical rail travel: 1.0 m
- Vertical rail speed: up to 0.3 m/s
- YAM arms
- Jetson Thor is the deployment target

## Navigation sensor target
- One RealSense D435i depth camera, likely mounted overhead.
- Face robot +X, through the 0.542 m-wide end of the current URDF.
- Current URDF axes: X is the 0.751 m length, Y is the 0.542 m width, Z is up.
- Depth FOV is 87 degrees horizontal by 58 degrees vertical, with independent pinhole intrinsics for both angles.
- Camera X is centered (0 m); maximum height is 2.0 m above ground with the elevator fully raised. The default reference position is (0, 0, 2.0) m with zero roll/yaw. Y remains provisionally centered.
- Camera IMU fusion defaults to use_imu:=true. Gazebo's physics IMU shares the camera mount pose and requests gyro/accel at 100 Hz without rendering. Simulation publishes /camera/imu with unavailable orientation and explicit noise covariance; no absolute simulator attitude enters navigation. use_imu:=false restores the wheel-only simulation bridge.
- camera_pitch:=auto puts the lower edge of the view on the ground 1 m beyond the front edge. At 2.0 m height and 58-degree vertical FOV this is about 26.48 degrees down. Height, both FOVs, resolution, frame rate, and ground distance are adjustable.
- The camera moves with the elevator on hardware. The current simulation uses a fixed joint at the selected camera_z; it does not animate elevator motion. Auto pitch is a launch-time calculation, not a model of an actuated camera tilt.
- Gazebo sensing defaults to sensor:=camera camera_backend:=cpu; camera_backend:=render selects the optional rendered depth camera. sensor:=lidar is a legacy option. Launch files share the camera mount pose with the URDF.
- CPU camera mode uses the robot_cpu_camera Gazebo system and DART/Bullet physics ray queries against current collision geometry, including inserted/moving objects. It publishes synthetic PointCloud2 through the existing depth_points_bridge and /scan only through camera_to_scan. No rendered depth sensor, lidar sensor, depth images, or CameraInfo are active in CPU mode.
- CPU sampling defaults to 80x45 rays at camera_fps (30 Hz requested simulation time), adjustable with camera_ray_width/camera_ray_height. This is separate from the physical/rendered 1280x720 image profile. It approximates coverage and occlusion; sparse rays and ideal collision geometry do not reproduce all RealSense detections/noise/holes.
- CPU world generation replaces the ground-plane collision with a 0.2 m slab whose top is the original plane and whose X/Y extent matches the visible floor (100x100 m by default). This avoids inaccurate raised-floor ray hits from DART's internal 2100 m plane box. Source SDF and visuals are unchanged; larger worlds need appropriately sized floor collisions.
- Default depth resolution is 1280x720 at a requested 30 Hz, with 0.3-3.0 m depth clipping, matching the supplied D435i specifications. camera_image_width/camera_image_height/camera_fps adjust it; actual throughput depends on rendering and transport capacity. colcon.meta builds the scan converter in Release mode.
- Only camera_backend:=render needs the workspace-local Gazebo Rendering 8.2.3 projection correction. Build it with `python3 src/robot_simulation/scripts/build_camera_renderer.py`. CPU mode and physical hardware do not use it. `scripts/check_cpu_camera.py` checks CPU ray geometry/occlusion; `scripts/check_camera_projection.py` checks rendered cloud geometry.
- Simulation launches use a fresh GZ_PARTITION unless explicitly supplied and bridge the selected world's clock to ROS /clock. gui:=false with CPU sensing runs without a rendering backend; GUI/RViz displays still render when enabled.
- Interactive use requires gui:=true so the user can inspect the world and insert obstacles. Keep it true in final run commands and performance validation; do not use headless mode as a solution to poor interactive performance.
- On GPU-capable WSL, Gazebo/RViz automatically select Mesa D3D12 graphics unless the user explicitly selects a driver/software rendering. This is launch-local environment configuration; navigation, EKF and CPU camera remain CPU-based. Native Linux/Jetson driver selection is untouched.
- Generated mazes combine adjacent occupied cells in each row into equivalent wall boxes, reducing rendering/physics geometry without changing the map or wall coverage. Visible-GUI seed 1400869175 checks improved from about 0.34 to 0.60 real-time factor; full real-time speed is not established.
- Physical sensing remains D435i -> RealSense ROS driver -> PointCloud2 -> camera_to_scan -> /scan -> Nav2. No physical RealSense bringup launch is present in this checkout; hardware driver integration/calibration must not be reported as tested here.
- Transform depth points into level camera_scan_frame at Z=0.22 m and retain points 0.10-1.00 m above ground. Missing returns remain unknown; do not clear unseen areas as free space.
- Keep legacy laser_link for the selectable lidar simulation mode.

## Navigation Architecture
- Use Nav2 as the navigation framework.
- SLAM provides map data and map -> odom.
- robot_state_estimation uses robot_localization EKF to fuse wheel vx/vy/yaw rate and all camera gyro axes, transformed into the base frame. It publishes /odom and owns odom -> base_footprint; the URDF connects base_link. The raw Gazebo odometry TF is not bridged while fusion is enabled. AMCL still owns map -> odom.
- D435i IMU has no absolute map position/yaw reference. Only gyro measurements are fused; camera orientation and acceleration are excluded to avoid gravity/elevator/arm-motion artifacts. Physical driver enable_gyro/enable_accel and unite_imu_method:=2 supply the combined IMU topic; camera extrinsics, covariances and wheel uncertainty need hardware calibration.
- Nav2 outputs geometry_msgs/Twist commands.
- A ROS 2 bridge will convert /cmd_vel into I2RT FlowBaseClient commands.
- Navigation owns vx, vy, and yaw rate.
- VLA/manipulation owns the rail and arms.
- Keep navigation CPU-oriented where practical so GPU resources remain available for VLA and SLAM.
- Default global planning uses SmacPlannerHybrid (forward Dubins, 72 heading bins, 0.4 m minimum path radius), matching normal forward camera-guided travel. Hybrid A* is curvature constrained and does not plan pure sideways motion.
- Default local control uses CPU-based MPPI Omni, a 2.4 s horizon and 1000 trajectories. Forward and lateral limits are 0.63 m/s, reverse is 0.10 m/s and yaw is +/-0.8 rad/s. PathAngle/PreferForward and path-orientation scoring turn the robot toward its route. Twirling weight is reduced to 0.5 so necessary heading changes are allowed.
- robot_mppi_critics/ForwardTravelCritic penalizes body-lateral travel outside a 15-degree forward cone on clear paths (weight 60). It relaxes that preference (weight 4) only for a known forward obstacle within 1.5 m while facing the route within 0.5 rad. This is optimizer scoring, not post-controller clipping. Normal clear-path travel should face forward; Omni sidesteps remain available for avoidance and small corrections.
- The velocity chain is cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> planar_velocity_limiter -> cmd_vel_limited -> collision_monitor -> cmd_vel. The limiter caps total translation hypot(vx, vy) to 0.63 m/s, including diagonals, and sends zero on stale/nonfinite commands. Maximum speed is a ceiling, not a guarantee near obstacles/goals.
- AMCL uses likelihood_field_prob with beam skipping, 0.05 m/rad update thresholds, and low simulation odometry noise (alpha1/3=0.002, alpha2/4/5=0.0005). The inserted-obstacle validation passed after this adjustment; recalibrate before hardware use. Camera-derived /scan remains the localization input.
- NavigateToPose keeps a valid global path and replans on blockage or a changed goal. Both default navigation behavior trees recover using costmap clearing, waiting and slow backup; Spin recovery is removed. Global inflation radius 1.50 m and Hybrid A* cost_penalty 2.0 favor clearance in 2.5 m corridors.
- The forward camera has side/rear blind spots during holonomic travel. Camera-only sensing does not establish safe full-speed movement in unobserved directions. Recent long-maze runs showed AMCL drift; changing planners/controllers does not fix localization or slow simulation time.
- `src/robot_navigation/scripts/validate_holonomic_navigation.py` tests three Hybrid A* paths and three actual navigation goals, optionally with `--dynamic-obstacle`. Evidence is in `Progress_Reports/holonomic_navigation_2026-10-28/`. Ground truth is used only for checking results, not as a navigation input. Controlled tests and a short maze goal passed; full-maze reliability remains open.
- RViz launch default is robot_navigation/config/flow_navigation.rviz: TopDownOrtho, near clip 0.01, target <Fixed Frame>, scale 18, angle -1.5725, X=0, Y=0.

## Packages
- robot_description: URDF/Xacro, meshes, frames
- robot_simulation: Gazebo worlds and simulation
- robot_cpu_camera: rendering-free Gazebo camera approximation using physics ray casting
- robot_state_estimation: common CPU wheel/IMU EKF configuration and standalone launch
- robot_mppi_critics: forward travel preference inside the MPPI optimizer
- robot_navigation: Nav2 config, BTs, planners/controllers
- robot_base_bridge: I2RT base interface
- robot_bringup: top-level launch files

## Current roadmap
1. Create simplified URDF of FLOW base. ##Done
2. Make simulated base holonomic in Gazebo.  ##Done
3. Connect Nav2 /cmd_vel to simulated base.  ##Done
4. Build robot_base_bridge using I2RT FlowBaseClient.  ##Done
5. Determine source of odom -> base_link.  ##Done
6. Drive real base slowly with Nav2.  ##Done
7. Integrate SLAM/nvblox map output.
8. Integrate rail/YAM/VLA handoff.

## Current task
The user has explicitly extended the camera work to Gazebo/Nav2 sensing integration.
Current focus: forward-facing travel on clear paths, holonomic MPPI obstacle avoidance, measured IMU/EKF/map-localization accuracy against timestamp-matched Gazebo truth, and a 0.63 m/s total translation limit. The user's latest preference supersedes unrestricted normal strafing: face the route and walk forward, with sideways motion for avoidance/corrections.
Hardware integration remains future work unless explicitly requested.
