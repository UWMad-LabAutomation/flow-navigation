# FLOW base navigation

This package starts the simulated holonomic FLOW base, a planar lidar, Nav2,
and RViz2 together. AMCL localizes against a saved occupancy-grid map,
the Smac 2D A* planner produces a global path, and the MPPI controller follows it using
the base's x, y, and yaw degrees of freedom.

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

The scan bridge publishes `/scan`, the simulator consumes Nav2's final
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
`maze_world.sdf`, `maze.yaml`, and `maze.pgm` are written to `/tmp/flow_maze`.
In RViz, set the initial pose near the generated start cell and send a goal
near the generated goal cell.
