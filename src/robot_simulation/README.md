# robot_simulation

This package runs the simplified FLOW base in Gazebo Harmonic. The model uses
Gazebo's `MecanumDrive` system as a first holonomic approximation of the
powered-caster base. It accepts forward velocity, lateral velocity, and yaw
rate commands.

## Launch

Source ROS 2 and the workspace, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch robot_simulation flow_base_sim.launch.py
```

The launch file starts Gazebo, bridges the simulated command and state topics,
and publishes the static `base_footprint` to `base_link` transform from the
robot description.

### Choose the sensing source

The default `sensor:=camera` uses a forward-facing
depth camera approximating the D435i:

```bash
ros2 launch robot_simulation flow_base_sim.launch.py sensor:=camera
```

Use `sensor:=lidar` explicitly for the legacy 360-degree scan. Camera mode publishes:

| ROS topic | Message type |
| --- | --- |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`, depth in metres) |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` |
| `/scan` | `sensor_msgs/msg/LaserScan`, projected from camera depth |

The image and camera information use `camera_depth_optical_frame`: Z forward,
X right, Y down. Gazebo Harmonic's point cloud uses X forward, Y left, Z up;
a separate bridge labels it `camera_depth_frame` to match those coordinates.
Both frames connect to the camera mount through the URDF.
Both sensing modes bridge `/clock`, `/odom`, `/tf`, and `/cmd_vel`.
Only the selected sensing source runs. In camera mode, `camera_to_scan`
transforms `/camera/depth/points` into the level `camera_scan_frame`, keeps
points 0.10 to 1.00 m above the floor, and projects their XY distances into
a forward scan with 0.5-degree bins. The simulated lidar is disabled.
The scan frame is at Z = 0.22 m for the existing AMCL/costmap/collision-monitor
pipeline; it is a virtual projection plane, not the camera's physical height.
Missing returns are reported above `range_max`, so they are discarded rather
than treated as observed free space. Obstacles below the 0.10 m height filter
and areas outside the camera view are not represented by this scan.

The ideal pinhole depth sensor uses 320 x 180 pixels at 10 Hz, 90 degrees
horizontal FOV, and clipping distances of 0.28 to 3.0 m. These are simulation
settings, not a full reproduction of RealSense noise, stereo failures, RGB,
or IMU behavior. The vertical FOV follows the aspect ratio (about 58.7 degrees
at the default horizontal FOV), approximating the D435i's nominal 58 degrees.
The reduced resolution keeps camera transport and scan conversion responsive.
Use `camera_image_width:=640 camera_image_height:=360` for higher resolution
when there is enough processing and transport capacity. Width and height also
feed the automatic pitch calculation, so changing aspect ratio retains the
requested ground coverage. Build from the workspace root to apply the
Release setting for the scan converter in `colcon.meta`.

The default camera reference height is provisionally 1.70 m, facing +X.
`camera_x`, `camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and
`camera_yaw` update both Gazebo sensing and the URDF transforms. Position
arguments are in metres and rotations in radians; positive pitch tilts down.
`camera_pitch:=auto` is the default. It calculates about 21.67 degrees down
at the default height, so the bottom of the view reaches the ground one metre
beyond the 0.751 m-long base's front edge. `camera_ground_distance` changes
that distance; changing height or FOV recalculates the tilt. The automatic
calculation assumes zero roll/yaw and level ground. An explicit numeric
`camera_pitch` overrides it.
`camera_hfov` is in degrees, while `camera_near` and `camera_far` are in metres.
For example, to use the requested approximate 90-degree horizontal view:

```bash
ros2 launch robot_simulation flow_base_sim.launch.py \
  sensor:=camera camera_hfov:=90 camera_z:=1.70
```

Use `gui:=false` for a server with headless rendering. The sensor still needs
a working Ogre2 rendering backend. The launch creates a configured world in
`/tmp/flow_sensors_*` and logs its path; it preserves the original world file.
The camera body and sensing pose are derived from the expanded URDF, so mount
changes are shared between Gazebo and ROS. Camera/bracket dynamics remain
outside this simplified model.
The generated world compensates for the SDF model origin being at wheel-center
height, so `camera_z` matches ROS's ground-level base frame on the flat floor.

Inspect the sensor after launching:

```bash
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /camera/depth/image_raw
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
The default 2.5 m corridors leave clearance for the 0.751 m by 0.542 m base
and the Nav2 inflation layer.
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

For a direct Gazebo-only check, use the equivalent Gazebo Transport command:

```bash
gz topic -t /model/flow_base/cmd_vel -m gz.msgs.Twist \
  -p 'linear: {y: 0.25}, angular: {z: 0.0}'
```

This is a dynamics approximation. The wheel collisions use anisotropic
friction rather than explicit roller geometry, and the base height and COM
remain provisional until measured hardware data is available.
