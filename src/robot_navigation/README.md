# FLOW base navigation

This package starts the simulated holonomic FLOW base, a tilted depth camera, Nav2,
and RViz2 together. AMCL localizes against a saved occupancy-grid map,
the Smac 2D A* planner produces a global path, and Regulated Pure Pursuit follows it
using forward motion and yaw.

Both launch files accept `sensor:=camera` (default) or legacy `sensor:=lidar` and
forward the camera pose, FOV, and clipping arguments to Gazebo. Camera mode
publishes depth images, camera information, a point cloud, and `/scan` projected
from height-filtered camera points. The lidar is disabled in camera mode.
See [the simulation README](../robot_simulation/README.md) for sensor topics
and adjustable ground coverage.

Default navigation drives +X through the short 0.542 m-wide end, with **zero
lateral velocity**. It follows the planned path at up to 0.35 m/s, rotates to
align when heading error exceeds 0.35 rad (about 20 degrees), and slows near
curves, obstacles, and the goal. Fixed 0.8 m lookahead and gentler acceleration
reduce abrupt steering corrections. The velocity smoother also enforces
zero lateral motion and limits yaw rate to 0.8 rad/s.

The launch keeps the current global path while it remains valid, checks it at
1 Hz, and replans when it is blocked or the goal changes. Nav2's recovery actions
remain enabled. Override `nav_to_pose_bt` to select another behavior tree.
The global inflation radius is 1.50 m with planner cost weight 4.0, so costs
extend to the centre of the default 2.5 m corridors and favor a straight route
with wall clearance. The controller still makes small heading corrections and
anticipates mapped corners; it does not wait to reach a wall before turning.

AMCL uses `likelihood_field_prob` with more frequent pose updates and motion
noise tuned for Gazebo's wheel odometry. The previous broad motion noise and
weak camera weighting could move the estimated pose into a wall, causing
`Start occupied`, excessive steering, or an empty local path. These odometry
noise settings need calibration for the real base. Localization still uses
the camera-derived `/scan`; simulator ground truth is not fed to Nav2.

The Gazebo base remains physically holonomic. The default path follower does
not reverse; the BackUp recovery behavior can still reverse slowly at up to
0.10 m/s. Costmap inflation, footprint collision checking, and the collision
monitor remain part of navigation.

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

The camera point-cloud converter publishes `/scan`, the simulator consumes Nav2's final
collision-monitored velocity on `/cmd_vel`, and the existing Gazebo bridge
publishes `/odom` and the `odom -> base_footprint` transform.

To generate a fresh maze with matching Gazebo walls and a map for Nav2:

```bash
ros2 launch robot_navigation maze_navigation_launch.py
```

The generator uses a 21 by 21 recursive-backtracking maze, 2.5 m corridors,
and 0.05 m map resolution. This leaves clearance for the FLOW footprint and
Nav2 inflation layer. The maze launch configures Smac 2D with a large search
budget for the long winding corridors. Use `seed:=42` for a repeatable maze. The generated
`maze_world.sdf`, `maze.yaml`, and `maze.pgm` are written to a unique directory
under `/tmp`, or to the explicitly supplied `output_dir`.
In RViz, set the initial pose near the generated start cell and send a goal
near the generated goal cell.
