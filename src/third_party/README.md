# Third-party ROS dependency

`pointcloud_to_laserscan` is the unmodified upstream ROS package, release
**2.0.2**, downloaded from:

https://github.com/ros-perception/pointcloud_to_laserscan/tree/2.0.2

It is built in this workspace because system package installation requires
administrator credentials. Its upstream BSD license and copyright notices
are preserved in the package. The project's own packages use Apache-2.0.

The camera launch uses `pointcloud_to_laserscan_node` to transform depth
points into a level frame, reject floor returns, and publish `/scan`.

Build from `~/robot_ws` so the workspace's `colcon.meta` is picked up:

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pointcloud_to_laserscan
source install/setup.bash
```

The metadata selects `CMAKE_BUILD_TYPE=Release` for this package. The converter
transforms every camera point; an unoptimized build can fall behind the camera
and trigger Nav2's stale-scan stop. This keeps the upstream source unmodified.
When building from another directory, pass `--metas ~/robot_ws/colcon.meta`.
