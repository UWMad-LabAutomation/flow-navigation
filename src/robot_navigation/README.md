# FLOW base navigation

This package starts the simulated FLOW base, camera-only sensing, Nav2 and
RViz. Smac Hybrid A* plans global paths; CPU-based MPPI in **Omni** mode follows
them with forward-facing clear-path travel and holonomic avoidance corrections.

Both launch files default to `sensor:=camera camera_backend:=cpu` and forward
camera pose, FOV, clipping, and ray-sampling arguments to Gazebo. CPU mode
publishes a synthetic point cloud from physics ray casting and `/scan` projected
from height-filtered camera points. No lidar or rendered camera sensor runs.
Use `camera_backend:=render` for the optional depth-image/CameraInfo renderer.
The physical-camera pipeline is unchanged. See
[the simulation README](../robot_simulation/README.md) for sensing limitations.

The controller uses 1000 candidate trajectories over 48 steps of 0.05 s
(a 2.4 s horizon), with full footprint collision checks. Both static-map
obstacles and live camera obstacles feed its local costmap. X/Y velocity limits
are +0.63/-0.10 m/s forward/reverse and +/-0.63 m/s lateral; yaw is
+/-0.8 rad/s, translation acceleration is 0.6 m/s²
and deceleration is 1.0 m/s². PathAngle/PreferForward critics and
path-orientation scoring now favor facing
the route and walking forward. The Twirling weight is 0.5, allowing necessary
heading changes. A workspace MPPI `ForwardTravelCritic` strongly penalizes
predicted lateral travel outside a 15-degree forward cone on a clear path. It
reduces this cost when a known obstacle blocks the front and the robot is facing
its route. Omni sidesteps remain available for avoidance and small corrections;
commands are not clipped sideways after optimization. See
[the critic package](../robot_mppi_critics/README.md) for the exact conditions.
Final goal heading can still require rotation.

The final translation command is capped at **0.63 m/s in any direction**.
Independent X/Y limits alone would allow a 0.89 m/s diagonal, so the
`planar_velocity_limiter` enforces `hypot(vx, vy) <= 0.63` after smoothing and
before collision monitoring. It preserves translation direction and stops on
stale or nonfinite commands. Maximum speed is a ceiling: obstacle clearance,
acceleration, path geometry and goal approach can require slower travel.

```text
MPPI / recovery -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
  -> planar_velocity_limiter -> /cmd_vel_limited
  -> collision_monitor -> /cmd_vel -> simulated FLOW base
```

Hybrid A* uses forward Dubins motion, 72 heading bins, a 0.4 m minimum path radius
and the rectangular footprint. This radius is a global-path preference, not a
physical restriction on FLOW. **Hybrid A* does not generate pure sideways
paths.** The requested combination provides curved global routes and holonomic
local avoidance. For full holonomic global planning, use Smac Lattice with omni
motion primitives instead; that is not the planner configured here.

NavigateToPose retains valid paths and replans when blocked or the goal changes.
Both default behavior trees recover with costmap clearing, a short wait and
0.10 m/s backup, without Spin recovery. Global inflation is 1.50 m with Hybrid
cost penalty 2.0. Override `nav_to_pose_bt` to select another single-goal tree.

AMCL skips scan beams inconsistent with the saved map and uses low motion-noise
coefficients for Gazebo's accurate odometry (alpha1/3=0.002, alpha2/4/5=0.0005).
These values require measurement and recalibration on hardware.
AMCL still uses camera-derived `/scan` and simulation odometry; simulator ground
truth is not fed to navigation. The AMCL adjustment passed a controlled test
with an obstacle inserted after planning. Earlier long-maze runs showed large
pose drift; full-maze reliability remains unverified. RViz cloud/TF timing and
a simulator running below real time are separate issues. A single forward-facing camera
also leaves side/rear blind spots: holonomic motion does not establish obstacle
coverage in every travel direction. Hardware integration/calibration and the
previously troublesome inserted-obstacle maze remain separate validation work.

The launch defaults to `config/flow_navigation.rviz`. Its view matches the
requested screenshot: **TopDownOrtho**, near clip **0.01**, target frame
**<Fixed Frame>**, scale **18**, angle **-1.5725**, X/Y **0**. The fixed frame
is `map`; the preset shows the robot, map, costmaps, `/scan`, and global path.
Use `rviz_config:=/absolute/path/to/custom.rviz` to override it.

Build and source the workspace:

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The default CPU camera needs only the normal colcon build. The optional
`camera_backend:=render` requires a separate renderer build to correct
Harmonic's custom-intrinsics projection for the 87 x 58 degree D435i view; see
[the simulation README](../robot_simulation/README.md#local-projection-correction).

Start navigation with an imported map. The YAML file must reference its image
using a path that is valid from the machine running ROS, and the map should
match the Gazebo environment.

```bash
ros2 launch robot_navigation navigation_launch.py \
  map:=/absolute/path/to/my_map.yaml
```

For a smoke test without a custom map, the launch file defaults to Nav2's
`tb3_sandbox.yaml`; replace it with a map made from this robot's world before
expecting AMCL to localize correctly.

In RViz2:

1. Set **Fixed Frame** to `map`.
2. Click **2D Pose Estimate** and click-drag the robot's actual starting pose.
3. Click **Nav2 Goal** and click-drag a reachable pose.
4. Watch the global and local costmaps, planned path, and the robot move in
   Gazebo.

Useful checks from another terminal:

```bash
source ~/robot_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
ros2 action list | grep navigate
```

The camera point-cloud converter publishes `/scan`, and the simulator consumes
Nav2's final collision-monitored velocity on `/cmd_vel`. By default, the
`robot_state_estimation` EKF fuses wheel velocities and the camera gyro to
publish `/odom` and `odom -> base_footprint`. AMCL still publishes `map -> odom`.
`use_imu:=false` restores raw wheel odometry for comparison. The tilted IMU is
transformed into the base frame; gyro, wheel X/Y and wheel yaw rate are fused,
while IMU orientation/acceleration are excluded. See
[state estimation](../robot_state_estimation/README.md) for physical D435i setup.

To generate a fresh maze with matching Gazebo walls and a map for Nav2:

```bash
ros2 launch robot_navigation maze_navigation_launch.py
```

The generator uses a 21 by 21 recursive-backtracking maze, 2.5 m corridors,
and 0.05 m map resolution. This leaves clearance for the FLOW footprint and
Nav2 inflation layer. The maze launch uses Smac Hybrid A* with a large search
budget for the long winding corridors. Use `seed:=42` for a repeatable maze. The generated
`maze_world.sdf`, `maze.yaml`, and `maze.pgm` are written to a unique directory
under `/tmp`, or to the explicitly supplied `output_dir`.
In RViz, set the initial pose near the generated start cell and send a goal
near the generated goal cell.


For interactive use, keep Gazebo and RViz visible and request 10 Hz CPU sensing:

```bash
ros2 launch robot_navigation maze_navigation_launch.py \
  camera_backend:=cpu gui:=true use_rviz:=true camera_fps:=10
```

`gui` defaults to true so you can insert obstacles. GPU-capable WSL launches
select Mesa D3D12 for the interfaces automatically, respecting explicit graphics
overrides. This avoids software rendering on the CPU without moving navigation
or CPU ray casting onto the GPU. Physical navigation needs neither Gazebo nor
its graphics renderer.

Reproduce planning and holonomic navigation checks in a separate ROS domain:

```bash
python3 src/robot_navigation/scripts/validate_holonomic_navigation.py \
  --output /tmp/flow_holonomic_check --domain 72
```

Choose an unused domain and a new output directory. Gazebo and RViz remain
visible during this check. The script creates a small
room with matching mapped obstacles, runs three independent start/goal planning
queries and three sequential navigation goals, and checks actual Gazebo motion,
rectangular footprint clearance, nonzero lateral travel and the final speed
limit. AMCL and the camera-only pipeline remain active. Ground truth is used
only to assess results. Outputs include `results.json`, `launch.log`, and
`paths_and_navigation.png` (a measured plot, not an RViz screenshot). This test
does not establish full-maze reliability or safe travel through unseen obstacles.

Add `--dynamic-obstacle` to insert an obstacle after the initial planning
queries and first goal. The saved map is not changed. The final configuration
passed all three goals in this test, with 0.048-0.291 m actual endpoint errors
and up to 0.413 m/s lateral commands. See the plots and JSON summaries in
`Progress_Reports/holonomic_navigation_2026-10-28/`. A short maze goal also
passed. The controller remains CPU-based; these results do not establish a
full-speed guarantee in clutter or full-maze/hardware reliability.


Verify clear-path startup, turning and forward motion with:

```bash
python3 src/robot_navigation/scripts/validate_holonomic_navigation.py \
  --output /tmp/flow_forward_start_check --clear-start
```

This course has mapped boundary walls and no interior obstacles. It checks
body-forward distance and whether the first 0.5 m of travel stays in the camera's
forward horizontal field of view. The normal `--dynamic-obstacle` course checks
that the preference still permits obstacle avoidance. Both modes compare EKF
odometry and AMCL map poses against timestamp-interpolated Gazebo truth.


The final clear-start test passed two goals with 98.8-99.9% body-forward travel.
The actual maze seed 1400869175 corner-start regression passed with 99.6%
body-forward travel and the first 0.5 m entirely within the forward camera
window. The inserted-obstacle course passed three goals while retaining lateral
corrections. Evidence is in `Progress_Reports/forward_navigation_2026-10-28/`.
A post-goal collision-monitor shutdown crash was observed with simultaneous
SIGINT; the validator now deactivates Nav2 before stopping clock/TF.
