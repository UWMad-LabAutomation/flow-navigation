# robot_description

This package contains the initial simplified URDF/Xacro description of the
I2RT LinearBot FLOW base.

The model currently provides the navigation frames `base_footprint` and
`base_link`, along with a box visual, collision geometry, and inertial data.
The documented 0.751 m by 0.542 m footprint and approximately 80 kg mass are
used. The base-only height is not documented yet, so the model uses a
provisional 0.200 m height. The center of mass is also undetermined and is
provisionally placed at the geometric center of that box.

The inertia is calculated for a uniform rectangular solid. Replace the
provisional height, center of mass, and inertia with measured values before
using this model for dynamic simulation or hardware control.

## Forward-facing depth camera (Step 1)

The intended navigation sensor is one RealSense D435i mounted overhead and
looking forward along **+X** with downward pitch. In this URDF, X runs along the 0.751 m
base length, Y along the 0.542 m width, and Z upward. The camera looks through
the center of the 0.542 m-wide end of the base.

The simplified camera enclosure is 0.025 m deep (X), 0.090 m wide (Y), and
0.025 m high (Z). `camera_link` represents the nominal left-depth-imager
reference, with ROS body axes: X forward, Y left, Z up. The box center is
offset 0.0175 m toward camera right from this reference, following the
nominal RealSense description. This is approximate geometry, not a measured
mount or a calibrated sensor model. Camera/bracket mass, inertia, and bracket
geometry are not modeled yet.

The default camera reference pose is **(0, 0, 1.70) m** relative to
`base_link`, with zero roll/yaw and automatically calculated downward pitch.
The default is **0.37813 rad (21.67 degrees)**: the bottom edge of the
320 x 180, 90-degree horizontal view reaches the ground **1 m beyond the
front edge**, at X = 0.3755 + 1.0 = 1.3755 m relative to the base center.
The camera's center ray looks farther ahead. This calculation assumes level
ground and zero camera roll/yaw. The 1.70 m height and centered
placement are **provisional visualization values**, not confirmed mounting
measurements. The camera is currently fixed to the base; a mount on the
moving rail would require a different joint/frame arrangement.

Adjust the pose with these Xacro arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `camera_x` | `0.0` | Forward offset in metres |
| `camera_y` | `0.0` | Leftward offset in metres |
| `camera_z` | `1.70` | Height of the camera reference above `base_link`, in metres |
| `camera_roll` | `0.0` | Roll in radians |
| `camera_pitch` | `auto` | Calculate ground coverage, or specify downward pitch in radians |
| `camera_yaw` | `0.0` | Yaw in radians; zero faces robot +X |
| `camera_hfov` | `90.0` | Simulated horizontal FOV in degrees, used to calculate vertical coverage |
| `camera_image_width` | `320` | Depth image width in pixels, used with height to determine vertical FOV |
| `camera_image_height` | `180` | Depth image height in pixels |
| `camera_ground_distance` | `1.0` | Desired ground distance beyond the front edge, in metres |
| `camera_nominal_depth_frames` | `true` | Include nominal depth and optical transforms for description-only use |

For example, to inspect a different height and downward tilt:

```bash
source /opt/ros/jazzy/setup.bash
xacro src/robot_description/urdf/flow_base.urdf.xacro \
  camera_z:=1.50 camera_pitch:=0.174533 | check_urdf /dev/stdin
```

The simulation, navigation, and maze launch files expose the same pose
arguments and apply them to both URDF transforms and Gazebo sensing. Select
the simulated depth camera with `sensor:=camera`; see
[the simulation README](../robot_simulation/README.md).

The description provides this camera frame chain:

```text
base_link
  -> camera_link
    -> camera_depth_frame
      -> camera_depth_optical_frame
```

The optical frame uses Z forward, X right, and Y down. With the default mount,
its +Z axis points forward and downward. A separate `camera_scan_frame` is
fixed to `base_link` at the camera's X/Y position and Z = 0.22 m, with zero
roll/pitch and the camera's yaw. It provides a level plane for `/scan` after
transforming the 3D points and filtering out the floor.
Before integrating the hardware driver,
set `camera_nominal_depth_frames:=false` if the driver supplies these internal
transforms, and match its frame names to this mount. RGB and IMU frames are
outside this initial depth-camera description.

The requested **90-degree horizontal view (+/-45 degrees)** is a planning
approximation. The D435i's nominal depth FOV is **87 degrees horizontal by
58 degrees vertical** (about +/-43.5 degrees horizontally). View angles
originate at the camera, not at the edges of the robot. Actual coverage also
depends on mounting height, tilt, and the active camera profile/calibration.
URDF frames and geometry do not configure a sensor's field of view or
generate depth data. The Gazebo launch handles this separately and exposes
`camera_hfov` in degrees, defaulting to the requested approximate 90 degrees.

The legacy `laser_link` remains for explicit `sensor:=lidar` use. The default
`sensor:=camera` activates only depth sensing in Gazebo and projects its
point cloud into `/scan` for AMCL, costmaps, and the collision monitor.

References:

- [RealSense specifications](https://www.realsenseai.com/compare-depth-cameras/)
- [Nominal RealSense D435 geometry and depth frames used by D435i](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d435.urdf.xacro)
- [ROS coordinate conventions (REP-103)](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
