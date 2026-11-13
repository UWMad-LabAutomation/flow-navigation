# robot_simulation

This package runs the simplified FLOW base in Gazebo Harmonic. The model uses
Gazebo's `MecanumDrive` system as a first holonomic approximation of the
powered-caster base. It accepts forward velocity, lateral velocity, and yaw
rate commands.

The source SDF base now matches the supplied 0.54 x 0.54 m X/Y dimensions.
It retains a provisional 0.20 m body height, 0.08 m ground clearance and
80 kg base mass. Approximate wheel centers are X/Y=+/-0.19 m with 0.08 m
radius. The world generator copies the fixed upright and 0.10 x 0.75 x 0.05 m
rail from the URDF. The rail is centered at `(0.20, 0, 1.25)` m with its length
along Y; +X remains forward. Its front face is 2 cm behind the base front.
The user-selected upright is 0.20 x 0.15 x 0.945 m, centered at
`(0.15, 0, 0.7525)` m, connecting base top Z=0.28 m to rail bottom Z=1.225 m
with the front faces flush. Its dimensions are a modeling choice, not measured
hardware. Upper-structure masses, elevator motion
and YAM arms are not modeled. The camera enclosure is visual-only in Gazebo
to avoid rays hitting their own housing; Nav2's footprint still encloses it.
Chassis, upright and rail collision shapes match the URDF. This geometry update is not physical dynamics
validation; see [robot_description](../robot_description/README.md).

## Launch

Build the workspace, including the `robot_cpu_camera` plugin:

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_navigation
```

Source ROS 2 and the workspace, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch robot_simulation flow_base_sim.launch.py
```

The launch file starts Gazebo, bridges the simulated command and state topics,
and publishes the static `base_footprint` to `base_link` transform from the
robot description.

### Default CPU camera

`sensor:=camera camera_backend:=cpu` uses physics ray casting against the
current collision geometry. It publishes synthetic `PointCloud2` on the same
topic as the rendered camera, so the existing `camera_to_scan` node remains
the only publisher of `/scan`. No lidar sensor, depth-image renderer, or local
renderer patch is loaded. Dynamic objects and newly inserted collision
objects are included; no map or ground-truth pose is sent to Nav2.

The generated CPU world uses a thin floor collision slab with the same top
surface and visible 100 x 100 m extent. This avoids false raised-floor ray
returns from DART's oversized internal plane box. The source SDF and floor
visual stay unchanged. Worlds extending beyond that area need a larger floor.

The default **80 x 45 ray grid** samples the **90 x 65 degree** camera view at
the requested **30 Hz of simulation time**, with **0.25-6.0 m optical
depth limits** and URDF mount. `camera_ray_width` and `camera_ray_height` change
sampling density independently of the physical/rendered 1280 x 800 profile.
Higher density detects finer geometry but costs more CPU time. Rays pass
through pixel centers, and only the closest hit is returned. Missing and
out-of-range points are NaNs, including when an object inside the near limit
occludes a farther object.

This is an ideal geometric approximation of the Gemini 335L view.
Thin objects can fall between rays; collision meshes may differ from visible
surfaces. Active-stereo depth holes, noise, illumination, material behavior,
USB transport and RGB behavior are not simulated. A separate physics IMU supplies
gyro/accel measurements. The planned physical pipeline is Gemini 335L -> Orbbec
ROS driver -> PointCloud2 -> camera_to_scan -> /scan -> Nav2, without Gazebo.
This checkout has no physical Orbbec bringup launch;
hardware calibration and validation remain separate work.

For interactive navigation, keep both Gazebo and RViz visible:

```bash
ros2 launch robot_navigation maze_navigation_launch.py \
  sensor:=camera camera_backend:=cpu gui:=true camera_fps:=10
```

`gui` defaults to true so obstacles can be inserted and moved. On GPU-capable
WSL, launch selects Mesa D3D12 for Gazebo and RViz instead of CPU software
rendering. Explicit driver/software overrides are respected; system libraries
are unchanged. The launch logs `FLOW WSLg GUI graphics: Mesa D3D12 GPU rendering`
when selected. This uses the GPU for the visible interfaces; ray casting,
navigation and state estimation remain CPU-based. The real robot needs no
Gazebo rendering.

Each launch selects a unique Gazebo partition unless `GZ_PARTITION` is already
set, and bridges the selected world's clock into ROS `/clock`. This prevents
an older Gazebo server from supplying a different clock or camera stream.
To use external `gz` commands, set the same explicit `GZ_PARTITION` in both
terminals before launching. Stop the previous ROS launch before starting a
new one; Gazebo partitioning does not isolate ROS topic names.

Generated maze walls merge occupied row runs into equivalent boxes. For seed
1400869175 this reduces 242 collision/visual boxes to 126 without changing wall
coverage or the map. A same-seed check with Gazebo and RViz visible and 10 Hz
CPU sensing improved from 0.34 to 0.60 real-time factor. This machine still runs
the full maze slower than real time; 10 Hz simulation sensing was about 6 Hz
wall-clock sensing in that check. These are short measurements, not a guarantee
for every maze or inserted obstacle. These October 28 results used the previous
geometry and D435i profile, before the November 9, 2026 model/camera update.

Check geometry, clipping, dynamic occlusion, and throughput without graphics:

```bash
python3 src/robot_simulation/scripts/check_cpu_camera.py
```

See [the CPU plugin README](../robot_cpu_camera/README.md) for implementation details.

### Local projection correction

This section applies only to `camera_backend:=render`. Build the renderer once
if you want to use the optional depth-image simulation:

```bash
python3 src/robot_simulation/scripts/build_camera_renderer.py
ros2 launch robot_simulation flow_base_sim.launch.py camera_backend:=render
```

Gazebo Rendering 8.2.3 has a depth-camera projection defect with explicit
lens intrinsics: the point cloud expands even though CameraInfo reports the
requested FOV. This workspace backports the correction from
[gz-rendering PR #1390](https://github.com/gazebosim/gz-rendering/pull/1390),
associated with [gz-sensors #614](https://github.com/gazebosim/gz-sensors/issues/614).
The patch adapts its clipping-plane correction to Harmonic's `PreRender`
implementation and makes CameraInfo's projection agree with its intrinsics.

`scripts/build_camera_renderer.py` downloads the pinned 8.2.3 source archive,
verifies its SHA-256, applies `patches/gz-rendering8-depth-projection.patch`,
and builds with two compiler jobs by default (`--jobs` changes this).
It needs CMake, a C++ compiler, `patch`, and Gazebo Rendering 8 / Ogre2
development dependencies. These are installed on the current WSL system.
The downloaded upstream source retains its Apache-2.0 license and notices.
An existing archive can be supplied using `--archive /path/to/archive.tar.gz`.

Source and build outputs stay in `build/camera_renderer`; installation stays
in `install/robot_simulation/opt/camera_renderer`. No system libraries are
replaced. Rendered-camera launch prepends this renderer to `GZ_RENDERING_PLUGIN_PATH`
and `LD_LIBRARY_PATH` for its child processes
and requires the completed local build, preventing silent use of the affected
system renderer. This recipe targets the workspace's normal isolated colcon
install and Gazebo Harmonic 8.2.3 dependencies.

After sourcing `install/setup.bash`, verify actual rendered geometry:

```bash
python3 src/robot_simulation/scripts/check_camera_projection.py
```

The check creates a temporary wall at 2 m and verifies depth and point-cloud
coordinates at the center and all four image edges, plus CameraInfo K/P.
It starts its own isolated Gazebo instance and stops it on completion.

### Choose the sensing source

The default `sensor:=camera camera_backend:=cpu` uses a forward-facing
ray-based depth-camera approximation:

```bash
ros2 launch robot_simulation flow_base_sim.launch.py sensor:=camera
```

Use `sensor:=lidar` explicitly for the legacy 360-degree scan. Camera topics:

| ROS topic | Message type | Camera backend |
| --- | --- | --- |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`, depth in metres) | render only |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | render only |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | cpu and render |
| `/scan` | `sensor_msgs/msg/LaserScan`, projected from camera depth | cpu and render |

The image and camera information use `camera_depth_optical_frame`: Z forward,
X right, Y down. Gazebo Harmonic's point cloud uses X forward, Y left, Z up;
a separate bridge labels it `camera_depth_frame` to match those coordinates.
Both frames connect to the camera mount through the URDF.
Both sensing modes bridge `/clock` and `/cmd_vel`. With `use_imu:=true` (default),
raw wheel odometry is bridged to `/wheel/odometry/raw`, then given simulation
covariances on `/wheel/odometry`. A physics-based IMU at the camera mount feeds
`/camera/imu` at a requested 100 Hz. The CPU EKF publishes filtered `/odom` and
`odom -> base_footprint`; Gazebo's competing odometry TF is not bridged.
`use_imu:=false` restores direct wheel `/odom` and TF bridging. No image renderer
is needed for the IMU. See [state estimation](../robot_state_estimation/README.md).
Only the selected sensing source runs. In camera mode, `camera_to_scan`
transforms `/camera/depth/points` into the level `camera_scan_frame`, keeps
points 0.10 to 1.35 m above the floor, and projects their XY distances into
a forward scan with 0.5-degree bins. The simulated lidar is disabled.
The scan frame is at Z = 0.22 m for the existing AMCL/costmap/collision-monitor
pipeline; it is a virtual projection plane, not the camera's physical height.
Missing returns are reported above `range_max`, so they are discarded rather
than treated as observed free space. Obstacles below the 0.10 m height filter
and areas outside the camera view are not represented by this scan. The upper
height bound includes obstacles at the fixed rail height; it does not establish
coverage for future moving arms or overhead structures.

The optional rendered pinhole depth sensor uses **1280 x 800 pixels at a requested 30 Hz**,
**90 degrees horizontal / 65 degrees vertical FOV**, and **0.25 to 6.0 m**
depth clipping. These are the Gemini 335L nominal depth profile and ideal range,
used as a simulation policy. The hardware's advertised 0.17-20 m+ working range
depends on profile and conditions; at 1280 x 800 the 0.25 m minimum requires
the 256 disparity search range. Independent lens focal lengths approximate
both FOVs. `camera_hfov` and `camera_vfov` control the angles.
`camera_image_width`, `camera_image_height`, and `camera_fps` control the
profile. FPS must be positive; FOVs must be between 0 and 180 degrees, image
dimensions must be positive integers, and `0 < camera_near < camera_far`.
The `/scan` period is set to `1 / camera_fps`, with a +/-45-degree default
angular window and 0.25-6.0 m planar range limits. Clipping is in camera depth;
scan ranges are horizontal distances after transforming the tilted cloud.
The requested rate is in simulation time; it does not guarantee 30 Hz of
wall-clock throughput. Build from the
workspace root to apply the scan converter's Release setting in `colcon.meta`.

The physical Gemini 335L uses active stereo, includes RGB and a 6-axis IMU,
and connects over USB 3.
Both simulation backends approximate ideal depth; stereo matching failures,
USB transport and RGB behavior are not modeled. The optional physics IMU models noisy gyro/accel
measurements, without claiming real-device calibration or exporting absolute
simulator attitude. Its enclosure is 124 x 29 x 27 mm (width x height x depth).
See the [official specifications](https://store.orbbec.com/products/gemini-335l)
and [datasheet v1.6](https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf).

The default camera reference is **(0.275, 0, 1.25) m**, facing +X from
2.5 cm ahead of the rail's front face. This standoff is provisional and keeps
the rail out of view.
The centered enclosure/optical reference is approximate; hardware needs the
left-IR optical origin and bracket transform from measurement/calibration.
`camera_x`, `camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and
`camera_yaw` update both Gazebo sensing and the URDF transforms. Position
arguments are in metres and rotations in radians; positive pitch tilts down.
`camera_pitch:=auto` is the default. It calculates about 18.98 degrees down
at the default height, so the bottom of the view reaches the ground one metre
beyond the 0.54 m-long base's front edge. `camera_ground_distance` changes
that distance; changing height or vertical FOV recalculates the tilt. The automatic
calculation assumes zero roll/yaw and level ground. An explicit numeric
`camera_pitch` overrides it. Camera pose overrides hold a fixed pose; they do
not move or resize the fixed rail and do not animate elevator travel or tilt.
They do not resize Nav2's fixed footprint either. Update that footprint if a
changed camera position or orientation extends beyond the default envelope.
Both FOVs are in degrees; `camera_near` and `camera_far` are in metres.
For example, to make the Gemini 335L rail-mount profile explicit:

```bash
ros2 launch robot_simulation flow_base_sim.launch.py \
  sensor:=camera camera_hfov:=90 camera_vfov:=65 camera_x:=0.275 camera_z:=1.25 \
  camera_image_width:=1280 camera_image_height:=800 camera_fps:=30 gui:=true
```

Keep `gui:=true` for interactive use. CPU sensing itself needs no renderer;
the visible Gazebo scene still uses graphics.
The launch creates a configured world in
`/tmp/flow_sensors_*` and logs its path; it preserves the original world file.
The camera body and sensing pose are derived from the expanded URDF, so mount
changes are shared between Gazebo and ROS. Camera/bracket dynamics remain
outside this simplified model.
The generated world compensates for the SDF model origin being at wheel-center
height, so `camera_z` matches ROS's ground-level base frame on the flat floor.

Inspect the sensor after launching:

```bash
ros2 topic hz /camera/depth/points
ros2 topic info /scan -v
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

In RViz, use `odom` as the fixed frame and add a PointCloud2 display on
`/camera/depth/points`. On an empty ground-plane world, the camera sees the
floor but `/scan` should contain no valid obstacle ranges after filtering.

For a random maze test, use the navigation launch. It generates a Gazebo
world with collision walls and a matching Nav2 occupancy map before starting
the simulator:

```bash
ros2 launch robot_navigation maze_navigation_launch.py
```

Use `seed:=42` to reproduce a maze, or omit the seed for a new maze each run.
The default 2.5 m corridors leave clearance for the revised fixed envelope
(Nav2 X=-0.27 to +0.30 m, Y=+/-0.375 m plus 0.02 m padding) and inflation layer.
Generated files are written to a unique directory under `/tmp` unless an
explicit `output_dir` is supplied.

The navigation and maze launch files forward the same sensor arguments.
The tilted camera can see the default 0.5 m-high maze walls:

```bash
ros2 launch robot_navigation maze_navigation_launch.py \
  sensor:=camera
```

AMCL, costmaps, and the collision monitor now receive the camera-derived
`/scan`. In RViz, set the initial pose at the generated start before sending
a goal. Forward-view localization has less coverage than the old 360-degree
lidar and still needs scenario testing; backward/sideways motion has limited
observability. Nav2 favors +X travel while allowing slow holonomic adjustments.

The standard `pointcloud_to_laserscan` 2.0.2 dependency is included under
`src/third_party` with its upstream BSD license. Build it with the workspace:

```bash
colcon build --symlink-install --packages-up-to robot_navigation
```

## Manual motion checks

In another terminal, source the same environments and send one command at a
time. Each command should run for about two seconds, followed by a zero command
to stop the base:

```bash
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.25}, angular: {z: 0.0}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

The first command should move along +X, the second should translate sideways
along +Y without a turn, and the third should rotate in place. Inspect the
resulting odometry with:

```bash
ros2 topic echo /odom --once
ros2 topic echo /tf --once
ros2 topic hz /odom
```

For a direct Gazebo-only check, use the equivalent Gazebo Transport command
in a terminal sharing the launch's explicit `GZ_PARTITION`:

```bash
gz topic -t /model/flow_base/cmd_vel -m gz.msgs.Twist \
  -p 'linear: {y: 0.25}, angular: {z: 0.0}'
```

This is a dynamics approximation. The wheel collisions use anisotropic
friction rather than explicit roller geometry, and the base height and COM
remain provisional until measured hardware data is available.
