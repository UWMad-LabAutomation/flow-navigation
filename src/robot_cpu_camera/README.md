# FLOW CPU camera

This Gazebo Harmonic system approximates an ideal depth camera by casting
pinhole rays against the physics world's collision geometry. It publishes
`gz.msgs.PointCloudPacked`, which the existing bridge converts to ROS
`PointCloud2`. It does not use a rendering sensor, a GPU lidar, or a map as
sensor input. It is only loaded by the simulation launch.

Build normally with `colcon build --symlink-install --packages-up-to robot_navigation`.
The `robot_simulation` launch locates the installed plugin automatically.
Select `sensor:=camera camera_backend:=cpu` (the defaults).

The plugin creates a `RaycastData` component beneath the robot model and lets
Gazebo's Physics system query current collision shapes. It selects the Bullet
collision detector within DART, which supports ray queries in the installed
Harmonic stack. Static and moving objects, including objects inserted after
startup, participate in occlusion. A visual without collision geometry is
not sensed. Detection accuracy also depends on collision-mesh fidelity.

The default grid is **80 x 45 rays at 30 Hz of simulation time**, adjustable
with `camera_ray_width`, `camera_ray_height`, and `camera_fps`. Ray density is
independent of the physical/rendered image profile, which remains 1280 x 720.
Independent pinhole focal lengths reproduce the requested 87 x 58 degree
viewing volume; samples pass through coarse pixel centers. Sparse rays can
miss objects smaller than their spacing. Performance depends on scene size
and CPU capacity; the requested simulation rate is not a wall-clock guarantee.

In CPU mode the generated world's `ground_plane` collision is a 0.2 m thick
slab whose top coincides with the original plane. Its X/Y extent matches the
plane's visible size (100 x 100 m in the supplied world); beyond that extent
there is no floor support. This avoids inaccurate ray hits from DART's internal
2100 m box representation of a plane, which otherwise creates false raised-floor
returns. The source SDF and floor visual are unchanged. Custom worlds should
provide appropriately sized floor collision geometry.

The generated plugin pose shares the URDF mount, including its height and
pitch, with compensation for the Gazebo model's wheel-center origin. Rays
start at the optical origin and stop at the far *depth plane*. The first hit
is retained only if its camera-X depth lies in 0.3-3.0 m. Too-near objects
still occlude farther objects. Missing/out-of-range returns are XYZ NaNs and
the organized cloud has `is_dense=false`. There is no artificial free-space
return and no direct `/scan` publisher in this plugin.

Cloud coordinates use `camera_depth_frame`: X forward, Y left, Z up. Frames
are stamped with the physics simulation time for the same update. Ray work
is disabled between camera updates and while physics is paused. The existing
`camera_to_scan` converter handles TF, floor rejection, and planar projection.

This is an ideal geometric approximation. It does not reproduce active-stereo
matching, illumination, material effects, USB latency, noise, or RealSense
depth holes. Hardware still uses the RealSense driver and its calibrated
frames, followed by `PointCloud2 -> camera_to_scan -> /scan -> Nav2`.

Validation:

```bash
colcon test --packages-select robot_cpu_camera robot_simulation
python3 src/robot_simulation/scripts/check_cpu_camera.py
```

The live check runs without DISPLAY or headless rendering. It checks a
collision-only wall, both FOVs, frame conventions, dynamic insertion/movement,
near/far clipping, first-hit occlusion, and publication rate. Test worlds use
their own Gazebo partition and are stopped when the check finishes.
