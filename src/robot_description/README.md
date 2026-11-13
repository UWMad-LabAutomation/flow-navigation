# robot_description

This package contains the simplified URDF/Xacro description of the I2RT
LinearBot FLOW base. The November 9, 2026 update incorporates the user-supplied
base and arm-rail dimensions, with **+X forward**, +Y left and +Z up.

| Part | Geometry and placement |
| --- | --- |
| Base | 0.54 x 0.54 m in X/Y; provisional 0.20 m height, from Z=0.08 to 0.28 m |
| Arm rail | 0.10 x 0.75 x 0.05 m in X/Y/Z, centered at `(0.20, 0, 1.25)` m; length along Y |
| Upright | User-selected 0.20 x 0.15 x 0.945 m in X/Y/Z, centered at `(0.15, 0, 0.7525)` m; connects base top to rail bottom at Z=1.225 m |
| Camera enclosure | 0.027 x 0.124 x 0.029 m in camera X/Y/Z, in front of the rail |

The rail's front face at X=0.25 m is 2 cm behind the base front. The upright's
front face is flush with it and its base stays within the chassis. These
upright dimensions are a user-selected modeling choice, not measured hardware.

The base retains a provisional 80 kg mass and uniform-box inertia. Its center
of mass is at the box center; upper-structure masses are not guessed. The
upright and rail are fixed visual/collision geometry, without elevator motion
or YAM arms. The simulation generator copies these upper shapes into the
Gazebo world, while its source SDF defines the matching base and approximate
wheels. The camera enclosure has URDF collision geometry, but the generated
Gazebo world keeps it visual-only to avoid self-hits at the ray origin. Nav2's
conservative footprint still encloses the camera housing. Height, ground
clearance, caster geometry, mount details and dynamics
still require hardware measurement and validation.

## Forward-facing Gemini 335L

The Orbbec Gemini 335L faces **+X** with downward pitch. Its default reference
is **(0.275, 0, 1.25) m** relative to `base_link`, preserving 2.5 cm clearance
ahead of the rail's front face at X=0.25 m so it does not obstruct the view.
This mount offset and the
centered enclosure/optical reference are provisional. The real depth origin
is at the left IR imager; use measured mounting and calibrated driver
extrinsics for hardware. The former RealSense enclosure offset is removed.

Automatic pitch is **0.33127 rad (18.98 degrees)** with zero roll/yaw. The
bottom edge of the nominal 90 x 65 degree view reaches level ground 1 m beyond
the base front, at X=0.27+1.0=1.27 m. The center ray looks farther ahead.
This is a launch-time geometric calculation, not an actuated tilt or physical
calibration. Camera pose overrides hold a fixed pose and do not animate or
resize the rail. They also do not resize Nav2's fixed footprint: if a changed
position or orientation extends the housing beyond the default envelope,
update the footprint before navigation.

Adjust the pose with these Xacro arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `camera_x` | `0.275` | Provisional reference 2.5 cm ahead of the rail front, in metres |
| `camera_y` | `0.0` | Leftward offset in metres |
| `camera_z` | `1.25` | Camera reference height above the floor in metres |
| `camera_roll` | `0.0` | Roll in radians |
| `camera_pitch` | `auto` | Calculate ground coverage, or specify downward pitch in radians |
| `camera_yaw` | `0.0` | Yaw in radians; zero faces robot +X |
| `camera_hfov` | `90.0` | Simulated horizontal FOV in degrees |
| `camera_vfov` | `65.0` | Simulated vertical FOV in degrees, used to calculate ground coverage |
| `camera_image_width` | `1280` | Depth image width in pixels |
| `camera_image_height` | `800` | Depth image height in pixels |
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
  -> rail_support_link
  -> arm_rail_link
  -> camera_link
    -> camera_imu_frame
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
transforms, and match its frame names to this mount. The nominal simulation
IMU shares the camera mount; the physical driver's calibrated gyro transform
must replace it. RGB geometry is not included.

The configured depth FOV uses the Gemini 335L nominal specifications:
**90 degrees horizontal by 65 degrees vertical** (+/-45 degrees horizontally). View angles
originate at the camera, not at the edges of the robot. Actual coverage also
depends on mounting height, tilt, and the active camera profile/calibration.
URDF frames and geometry do not configure a sensor's field of view or
generate depth data. The Gazebo launch handles this separately and exposes
`camera_hfov` and `camera_vfov` in degrees. Independent lens focal lengths
preserve both FOVs when image resolution or aspect ratio changes.

Simulation uses 1280 x 800 at a requested 30 Hz and 0.25-6 m clipping, chosen
from Orbbec's ideal range. The advertised working range is 0.17-20 m+; minimum
depth depends on profile and disparity settings. Hardware intrinsics and
usable stereo coverage vary with calibration and distance.

The legacy `laser_link` remains for explicit `sensor:=lidar` use. The default
`sensor:=camera` activates only depth sensing in Gazebo and projects its
point cloud into `/scan` for AMCL, costmaps, and the collision monitor.

References:

- [Orbbec Gemini 335L specifications](https://store.orbbec.com/products/gemini-335l)
- [Gemini 330 series datasheet v1.6](https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf)
- [ROS coordinate conventions (REP-103)](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
