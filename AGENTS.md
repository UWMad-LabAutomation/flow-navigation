# Robot Navigation Project Context

Last reviewed: 2026-11-09. This file records the implemented simulation,
available evidence, known limitations, and agreed next steps. Simulation
results must not be presented as physical-robot validation.

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
- User-confirmed base plan dimensions: 0.54 x 0.54 m (2026-11-09).
- Fixed arm rail: 0.10 m along X, 0.75 m along Y, 0.05 m along Z; center (0.20, 0, 1.25) m. Its front face at X=0.25 m is 0.02 m behind the base front. The user-selected upright is 0.20 x 0.15 x 0.945 m in X/Y/Z, centered at (0.15, 0, 0.7525) m; it connects chassis top Z=0.28 m to rail underside Z=1.225 m with front faces flush. The upright dimensions are a modeling choice, not a claim of measured hardware.
- Vertical rail travel: 1.0 m
- Vertical rail speed: up to 0.3 m/s
- YAM arms
- Jetson Thor is the deployment target
- Base and rail dimensions above were supplied by the user. Base height (0.200 m), ground clearance (0.080 m), mass (80 kg), center of mass and box inertias remain provisional. Upper-structure masses are unknown; geometry changes do not validate the mass distribution or stability.
- The URDF describes the chassis, fixed upright/rail and camera. Gazebo wheel/contact geometry is separately defined in `src/robot_simulation/worlds/flow_base.sdf`; the generated world copies upright/rail visual and collision boxes from the URDF. MecanumDrive still approximates the real powered-caster base. Its provisional wheel centers are at X/Y +/-0.19 m, radius 0.08 m and wheelbase/separation 0.38 m, fitting the square base. Future updates must reconcile the URDF and SDF.

## Navigation sensor target
- One Orbbec Gemini 335L depth camera mounted on the front face of the rail, facing robot +X.
- Current axes: X is forward, Y follows the rail's 0.75 m length, Z is up. The base is 0.54 m square.
- Nominal depth FOV is 90 degrees horizontal by 65 degrees vertical, with independent pinhole intrinsics for both angles.
- Default camera reference is (0.275, 0, 1.25) m with zero roll/yaw, preserving a 0.025 m standoff ahead of the rail's front face to avoid occluding downward rays. This exact mounting offset and centered 0.027 x 0.124 x 0.029 m enclosure are approximations; hardware optical-center extrinsics require calibration.
- Camera IMU fusion defaults to use_imu:=true. Gazebo's physics IMU shares the camera mount pose and requests gyro/accel at 100 Hz without rendering. Simulation publishes /camera/imu with unavailable orientation and explicit noise covariance; no absolute simulator attitude enters navigation. use_imu:=false restores the wheel-only simulation bridge.
- camera_pitch:=auto puts the lower edge of the view on the ground 1 m beyond the 0.27 m base front edge. At the default mount and 65-degree vertical FOV this is about 18.98 degrees down. Height, both FOVs, resolution, frame rate, and ground distance are adjustable.
- Upright, rail and camera are fixed in simulation; camera pose overrides do not animate or reposition the rail. Auto pitch is a launch-time calculation, not an actuated camera tilt or elevator model.
- Gazebo sensing defaults to sensor:=camera camera_backend:=cpu; camera_backend:=render selects the optional rendered depth camera. sensor:=lidar is a legacy option. Launch files share the camera mount pose with the URDF.
- CPU camera mode uses the robot_cpu_camera Gazebo system and DART/Bullet physics ray queries against current collision geometry, including inserted/moving objects. It publishes synthetic PointCloud2 through the existing depth_points_bridge and /scan only through camera_to_scan. No rendered depth sensor, lidar sensor, depth images, or CameraInfo are active in CPU mode.
- CPU sampling defaults to 80x45 rays at camera_fps (30 Hz requested simulation time), adjustable with camera_ray_width/camera_ray_height. This is separate from the physical/rendered 1280x800 image profile. It approximates coverage and occlusion; sparse rays and ideal collision geometry do not reproduce all Gemini detections/noise/holes.
- CPU world generation replaces the ground-plane collision with a 0.2 m slab whose top is the original plane and whose X/Y extent matches the visible floor (100x100 m by default). This avoids inaccurate raised-floor ray hits from DART's internal 2100 m plane box. Source SDF and visuals are unchanged; larger worlds need appropriately sized floor collisions.
- Default depth resolution is 1280x800 at a requested 30 Hz, with 0.25-6.0 m simulation depth clipping chosen from Orbbec's ideal range, not the advertised maximum range. Minimum depth depends on the hardware disparity/profile setting. camera_image_width/camera_image_height/camera_fps adjust the simulation; actual throughput depends on rendering and transport capacity. colcon.meta builds the scan converter in Release mode. Sources: https://store.orbbec.com/products/gemini-335l and https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf.
- Only camera_backend:=render needs the workspace-local Gazebo Rendering 8.2.3 projection correction. Build it with `python3 src/robot_simulation/scripts/build_camera_renderer.py`. CPU mode and physical hardware do not use it. `scripts/check_cpu_camera.py` checks CPU ray geometry/occlusion; `scripts/check_camera_projection.py` checks rendered cloud geometry.
- Simulation launches use a fresh GZ_PARTITION unless explicitly supplied and bridge the selected world's clock to ROS /clock. gui:=false with CPU sensing runs without a rendering backend; GUI/RViz displays still render when enabled.
- Interactive use requires gui:=true so the user can inspect the world and insert obstacles. Keep it true in final run commands and performance validation; do not use headless mode as a solution to poor interactive performance.
- On GPU-capable WSL, Gazebo/RViz automatically select Mesa D3D12 graphics unless the user explicitly selects a driver/software rendering. This is launch-local environment configuration; navigation, EKF and CPU camera remain CPU-based. Native Linux/Jetson driver selection is untouched.
- Generated mazes combine adjacent occupied cells in each row into equivalent wall boxes, reducing rendering/physics geometry without changing the map or wall coverage. Visible-GUI seed 1400869175 checks improved from about 0.34 to 0.60 real-time factor; full real-time speed is not established.
- Planned physical sensing is Gemini 335L -> Orbbec ROS driver -> PointCloud2 -> camera_to_scan -> /scan -> Nav2. No physical Orbbec bringup launch is present; hardware driver integration/calibration must not be reported as tested here.
- Transform depth points into level camera_scan_frame at Z=0.22 m and retain points 0.10-1.35 m above ground, including rail-height obstacles. Missing returns remain unknown; do not clear unseen areas as free space.
- Keep legacy laser_link for the selectable lidar simulation mode.

## Navigation Architecture
- Use Nav2 as the navigation framework.
- Current launches use a saved/generated occupancy map from map_server and AMCL for map -> odom. SLAM integration is upcoming; it does not currently replace AMCL in these launches.
- robot_state_estimation uses robot_localization EKF to fuse wheel vx/vy/yaw rate and all camera gyro axes, transformed into the base frame. It publishes /odom and owns odom -> base_footprint; the URDF connects base_link. The raw Gazebo odometry TF is not bridged while fusion is enabled. AMCL still owns map -> odom.
- Gemini 335L's six-axis IMU has no absolute map position/yaw reference. Only gyro measurements are fused; camera orientation and acceleration are excluded to avoid gravity/elevator/arm-motion artifacts. The Orbbec driver uses enable_sync_output_accel_gyro:=true for /camera/gyro_accel/sample; remap the EKF's IMU input and use the driver's calibrated optical-frame TF. Camera extrinsics, covariances and wheel uncertainty need hardware calibration.
- IMU fusion improves short-term odometry; it does not replace AMCL or remove its initial-pose requirement. Keep one publisher for each TF relationship. A future SLAM mode should own /map and map -> odom while EKF retains odom -> base_footprint.
- Nav2 outputs geometry_msgs/Twist commands.
- A physical ROS 2 bridge is intended to convert /cmd_vel into I2RT FlowBaseClient commands. No robot_base_bridge source package or complete physical bringup launch is present in this checkout; do not claim it is delivered or validated here.
- Navigation owns vx, vy, and yaw rate.
- VLA/manipulation owns the rail and arms.
- Keep navigation CPU-oriented where practical so GPU resources remain available for VLA and SLAM.
- Default global planning uses SmacPlannerHybrid (forward Dubins, 72 heading bins, 0.4 m minimum path radius), matching normal forward camera-guided travel. Hybrid A* is curvature constrained and does not plan pure sideways motion.
- Both costmaps conservatively project the fixed base/rail/camera envelope: X [-0.27, 0.30] m, Y [-0.375, 0.375] m, with 0.02 m padding. The forward critic probes from X=0.32 m with lateral half-width 0.425 m. Future arm or camera poses outside this envelope require corresponding footprint changes.
- Default local control uses CPU-based MPPI Omni, a 2.4 s horizon and 1000 trajectories. Forward and lateral limits are 0.63 m/s, reverse is 0.10 m/s and yaw is +/-0.8 rad/s. PathAngle/PreferForward and path-orientation scoring turn the robot toward its route. Twirling weight is reduced to 0.5 so necessary heading changes are allowed.
- robot_mppi_critics/ForwardTravelCritic penalizes body-lateral travel outside a 15-degree forward cone on clear paths (weight 60). It relaxes that preference (weight 4) only for a known forward obstacle within 1.5 m while facing the route within 0.5 rad. This is optimizer scoring, not post-controller clipping. Normal clear-path travel should face forward; Omni sidesteps remain available for avoidance and small corrections.
- The velocity chain is cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> planar_velocity_limiter -> cmd_vel_limited -> collision_monitor -> cmd_vel. The limiter caps total translation hypot(vx, vy) to 0.63 m/s, including diagonals, and sends zero on stale/nonfinite commands. Maximum speed is a ceiling, not a guarantee near obstacles/goals.
- AMCL uses likelihood_field_prob with beam skipping, 0.05 m/rad update thresholds, and low simulation odometry noise (alpha1/3=0.002, alpha2/4/5=0.0005). The inserted-obstacle validation passed after this adjustment; recalibrate before hardware use. Camera-derived /scan remains the localization input.
- NavigateToPose keeps a valid global path and replans on blockage or a changed goal. Both default navigation behavior trees recover using costmap clearing, waiting and slow backup; Spin recovery is removed. Global inflation radius 1.50 m and Hybrid A* cost_penalty 2.0 favor clearance in 2.5 m corridors.
- The forward camera has side/rear blind spots during holonomic travel. Camera-only sensing does not establish safe full-speed movement in unobserved directions. Recent long-maze runs showed AMCL drift; changing planners/controllers does not fix localization or slow simulation time.
- `src/robot_navigation/scripts/validate_holonomic_navigation.py` tests three Hybrid A* paths and three actual navigation goals, optionally with `--dynamic-obstacle`. Evidence is in `Progress_Reports/holonomic_navigation_2026-10-28/`. Ground truth is used only for checking results, not as a navigation input. Controlled tests and a short maze goal passed; full-maze reliability remains open.
- The 2026-11-01 visible Gazebo/RViz inserted-obstacle course passed all three planning queries and all three navigation goals. Peak total command speed was 0.63 m/s, peak lateral command 0.1059 m/s, and measured lateral travel 0.488 m. camera_to_scan was the only /scan publisher. See `Progress_Reports/holonomic_navigation_2026-11-01/validation_summary.json`.
- Long-maze planning is an open issue: seed 3851856309 produced repeated "exceeded maximum iterations" failures with the current 1,000,000-iteration / 5 s settings. A goal approximately 12.5 m away geometrically requires about 198 m through corridor cell centers. The grid-only route check is not a proven drivable path. Test these cases before declaring Hybrid A* reliable on arbitrary mazes; a missing global path prevents MPPI path following.
- RViz launch default is robot_navigation/config/flow_navigation.rviz: TopDownOrtho, near clip 0.01, target <Fixed Frame>, scale 18, angle -1.5725, X=0, Y=0.

## Packages
- robot_description: URDF/Xacro, meshes, frames
- robot_simulation: Gazebo worlds and simulation
- robot_cpu_camera: rendering-free Gazebo camera approximation using physics ray casting
- robot_state_estimation: common CPU wheel/IMU EKF configuration and standalone launch
- robot_mppi_critics: forward travel preference inside the MPPI optimizer
- robot_navigation: Nav2 config, BTs, planners/controllers
- robot_base_bridge: planned I2RT base interface; source absent in this checkout
- robot_bringup: package scaffold; no implemented top-level hardware launch
- third_party/pointcloud_to_laserscan: vendored camera-cloud-to-scan converter; upstream BSD license retained

## Completed work and evidence

| Work | Implemented result and verification |
| --- | --- |
| Simplified robot model and simulation | User-confirmed square base and fixed rail dimensions added 2026-11-09, with connecting upright, consistent URDF/generated Gazebo geometry and conservative Nav2 footprint. Mass/inertia, wheel approximation and exact camera mount remain provisional. |
| Gemini 335L simulation profile | Replaces the prior D435i defaults with nominal 90 x 65 degree FOV, 1280x800 profile, 0.25-6 m clipping and a fixed rail mount. Optional rendered-camera projection patch/build/check tooling is retained. Earlier progress reports remain evidence for their original D435i model only. |
| CPU camera-to-scan integration | Physics rays sample current collision geometry, including inserted objects, and publish synthetic PointCloud2 through the existing scan converter. No LiDAR or rendered depth sensor runs in default camera mode. Geometry, clipping and occlusion checks are provided. |
| Hybrid A* planning | SmacPlannerHybrid forward Dubins configuration and footprint-aware path checks. Four independent start/goal pairs passed in a custom Gazebo obstacle map; actual labeled RViz screenshots and an overview are in `Progress_Reports/path_planning_evidence_2026-11-01/`. These screenshots show planning, not execution from all four starts or SLAM map creation. |
| Holonomic MPPI | Omni sampling, forward-travel preference critic, trajectory/acceleration tuning, total-speed limiter and spin-free recovery trees. Controlled navigation with an inserted obstacle passed with Gazebo and RViz visible. |
| IMU and odometry | Simulated camera IMU, covariance preparation, wheel/gyro EKF, single odometry TF owner, bias/fusion checks and timestamp-matched truth comparisons. A 24.035 s simulation test measured EKF maximum errors of 9.03 mm and 0.116 degrees; these are not hardware calibration results. See `Progress_Reports/forward_navigation_2026-10-28/`. |
| GUI and simulation performance | WSL D3D12 selection for visible interfaces, equivalent maze-wall box merging, and camera/scan timing checks. Same-seed short-window real-time factor improved from about 0.34 to 0.60. See `Progress_Reports/gui_rendering_2026-10-28/`. |
| Documentation and evidence organization | Package READMEs, progress report, source validation scripts, path/IMU/navigation JSON and screenshots. Planning evidence was moved out of src into Progress_Reports at the user's request; use its current location in links. |

The first November 9 model's build, geometry tests, CPU-camera checks and visible
Gazebo/RViz startup check are recorded in
`Progress_Reports/model_update_2026-11-09/README.md`. Those results predate the
subsequent rail orientation/placement, upright and camera correction. The latest
change record is `Progress_Reports/rail_update_2026-11-09/README.md`; consult its
recorded checks before claiming validation. Earlier navigation-course results
also used previous geometry.

Historical roadmap entries marked the physical bridge and real-base driving
as done. Their implementation/evidence is not present here. Preserve that
distinction instead of assuming hardware readiness from the old checklist.

## Agreed future steps

The 2026-11-09 base and rail dimensions and user-selected upright are incorporated. Remaining physical
measurements and observed behavior are still required before replacing the
provisional heights, wheel geometry, masses or inertias with hardware claims.

| Task | Owner / listed effort | Prerequisite and deliverable |
| --- | --- | --- |
| Complete measured robot model | ZG / 3 | Base plan and fixed rail geometry updated. Obtain remaining chassis height/clearance, caster geometry, upright cross-section, precise camera mounting and mass/COM measurements. Reconcile the URDF and Gazebo SDF and inspect in RViz. |
| Validate updated robot model and physics in Gazebo | ZG / 3 | After the measured model update: test forward/lateral motion, turning and obstacle clearance using updated geometry/inertias. Adjust Nav2 footprint, velocity/acceleration limits and motion parameters where measurements justify it. Deliver a short visible Gazebo demonstration. |
| Integrate SLAM | Upcoming; can progress alongside measurements | Connect the selected SLAM/map pipeline to Nav2, establish /map and map -> odom ownership, and provide a launch mode that avoids simultaneous AMCL/SLAM TF publishers. An OccupancyGrid publisher alone does not provide localization. Validate mapping, map alignment and navigation/replanning with live obstacles. No SLAM provider is selected as completed here. |
| Complete physical bringup and calibration | After required hardware information/integration is available | Wire Gemini 335L point cloud and IMU through the Orbbec driver, measured camera/elevator TF, wheel odometry and the FLOW CAN command bridge using real time. Calibrate covariances, footprint, speed limits and stop behavior before claiming sim-to-real validation. Existing simulation launch files always include Gazebo. |
| Include moving rail and YAM arms | Later | The fixed rail/upright boxes are present; articulated rail motion and arms are not. Add measured arm geometry, joints, collisions and changing sensor transforms; assess the occupied envelope and mass distribution across configurations. Establish navigation/manipulation control handoff. Navigation owns vx/vy/yaw; VLA/manipulation owns rail/arms. Validate the combined robot after integration. |

Measurement/observation notes should include chassis and caster dimensions and
positions, camera pose at lowered/raised elevator positions, actual wheel/base
odometry frames and rates, movement directions, speed/acceleration, turning
behavior, mass/COM information where available, and rail/arm clearances. Record
unknowns explicitly. Powered-caster behavior must be compared with the current
MecanumDrive approximation.

Continue investigating long-maze search limits, AMCL/map alignment, scan/TF
timing and visible simulation throughput. Preserve camera-only sensing and
forward-preferred normal travel while testing fixes. Keep runtime evidence in
Progress_Reports and distinguish measured outcomes from planned changes.

## Launch and documentation conventions

- The root `README.md` is the GitHub entry point for this repository. Keep its setup, commands, status and roadmap consistent with actual source and evidence.
- Normal WSL simulation: `ros2 launch robot_navigation maze_navigation_launch.py sensor:=camera camera_backend:=cpu camera_fps:=10 use_imu:=true gui:=true use_rviz:=true` from a sourced workspace. Empty seed generates a new maze; an explicit seed reproduces it.
- Stop the previous launch before starting another in the same ROS domain. GZ_PARTITION isolation does not isolate ROS topics. Use Ctrl+C on the owning launch; avoid broad kill commands that stop unrelated Python/Node/VS Code processes.
- Set AMCL's initial pose at the known spawn before sending navigation goals. IMU/EKF does not initialize absolute map pose. Startup/reset TF warnings and a global-planner search failure are separate diagnoses.
- Do not run the relocated one-off screenshot helper as the normal navigation launch. Some recorded absolute paths/session PIDs refer to the original capture session; the saved PNG/JSON files are the evidence.

## Current task
Maintain the working CPU-camera, Hybrid A* and forward-preferred Omni MPPI
simulation with the 2026-11-09 base/rail dimensions and Gemini 335L profile while
documenting completed work and open limits. Await remaining measurements before
replacing provisional hardware values. Prepare for
upcoming SLAM integration and eventual YAM-arm/rail integration without marking
either as implemented. Documentation work does not authorize changing runtime
configuration, inventing hardware values or publishing to GitHub.
