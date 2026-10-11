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
- Requested horizontal view: approximately 90 degrees, or 45 degrees to each side of forward.
- D435i nominal depth FOV is 87 degrees horizontal by 58 degrees vertical; 90 degrees is an approximation.
- Mounting height and offsets are unconfirmed. The description uses a provisional camera reference position of (0, 0, 1.70) m, zero roll/yaw, and automatic downward pitch.
- camera_pitch:=auto puts the lower edge of the 320x180 view on the ground 1 m beyond the front edge. At 1.70 m height and 90-degree horizontal FOV this is about 21.67 degrees down. Height, FOV, resolution, and ground distance are adjustable.
- The current description assumes a fixed mount. Attachment to the moving rail is not established.
- Gazebo sensing defaults to sensor:=camera; sensor:=lidar is a legacy option. Launch files share the camera mount pose with the URDF.
- Camera mode publishes depth images, CameraInfo, PointCloud2, and /scan from pointcloud_to_laserscan, with no lidar sensor running.
- Default depth resolution is 320x180 at 10 Hz to limit point-cloud transport load. camera_image_width/camera_image_height adjust it; colcon.meta builds the scan converter in Release mode to keep per-point transforms fast.
- Transform depth points into level camera_scan_frame at Z=0.22 m and retain points 0.10-1.00 m above ground. Missing returns remain unknown; do not clear unseen areas as free space.
- Keep legacy laser_link for the selectable lidar simulation mode.

## Navigation Architecture
- Use Nav2 as the navigation framework.
- SLAM provides map data and map -> odom.
- Base/state-estimation layer provides odom -> base_link.
- Nav2 outputs geometry_msgs/Twist commands.
- A ROS 2 bridge will convert /cmd_vel into I2RT FlowBaseClient commands.
- Navigation owns vx, vy, and yaw rate.
- VLA/manipulation owns the rail and arms.
- Keep navigation CPU-oriented where practical so GPU resources remain available for VLA and SLAM.
- Default navigation uses Regulated Pure Pursuit with forward motion and yaw, zero lateral velocity, and 0.35 m/s forward speed. Rotate to align with the path for heading errors above 0.35 rad. The base remains physically holonomic; slow reverse is reserved for BackUp recovery.
- AMCL uses likelihood_field_prob and 0.05 m/rad update thresholds with low motion noise calibrated for Gazebo wheel odometry; recalibrate before hardware use. Camera-derived /scan remains the localization input.
- NavigateToPose keeps a valid global path and replans on blockage or a changed goal, with Nav2 recoveries retained. Global inflation radius 1.50 m and Smac cost_travel_multiplier 4.0 favor the centers of 2.5 m corridors.
- RViz launch default is robot_navigation/config/flow_navigation.rviz: TopDownOrtho, near clip 0.01, target <Fixed Frame>, scale 18, angle -1.5725, X=0, Y=0.

## Packages
- robot_description: URDF/Xacro, meshes, frames
- robot_simulation: Gazebo worlds and simulation
- robot_navigation: Nav2 config, BTs, planners/controllers
- robot_base_bridge: I2RT base interface
- robot_bringup: top-level launch files

## Current roadmap
1. Create simplified URDF of FLOW base.
2. Make simulated base holonomic in Gazebo.
3. Connect Nav2 /cmd_vel to simulated base.
4. Build robot_base_bridge using I2RT FlowBaseClient.
5. Determine source of odom -> base_link.
6. Drive real base slowly with Nav2.
7. Integrate SLAM/nvblox map output.
8. Integrate rail/YAM/VLA handoff.

## Current task
The user has explicitly extended the camera work to Gazebo/Nav2 sensing integration.
Current focus: downward camera coverage at 1 m beyond the front edge, camera-derived /scan, straight path following through the short side without strafing, and the requested RViz defaults.
Hardware integration remains future work unless explicitly requested.
