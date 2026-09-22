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

For a random maze test, use the navigation launch. It generates a Gazebo
world with collision walls and a matching Nav2 occupancy map before starting
the simulator:

```bash
ros2 launch robot_navigation maze_navigation_launch.py
```

Use `seed:=42` to reproduce a maze, or omit the seed for a new maze each run.
The default 2.5 m corridors leave clearance for the 0.751 m by 0.542 m base
and the Nav2 inflation layer.
Generated files are written to `/tmp/flow_maze`.

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
