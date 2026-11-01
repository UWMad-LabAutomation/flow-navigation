# Gazebo map and actual RViz planning screenshots

This new folder contains a matching Gazebo obstacle world and 0.05 m occupancy map, four independent Smac Hybrid A* start/goal queries, and actual RViz window screenshots. All existing project files were left unchanged.

[Gazebo world screenshot](gazebo_scene.png) | [All four paths in RViz](all_paths_rviz.png)

| Case | Start (x, y, yaw degrees) | Goal (x, y, yaw degrees) | Path length | Actual RViz screenshot |
| --- | --- | --- | --- | --- |
| 1 - Across the central obstacles | (0, 0, 0) | (9, 0, 0) | 10.57 m | [Screenshot](case_01_rviz.png) |
| 2 - Lower left to upper right | (0, -3, 0) | (9, 3, 90) | 11.32 m | [Screenshot](case_02_rviz.png) |
| 3 - Upper corridor, heading west | (9, 3, 180) | (0, 3, 180) | 9.22 m | [Screenshot](case_03_rviz.png) |
| 4 - Lower right to upper left | (9, -3, 180) | (0, 2.8, 180) | 11.93 m | [Screenshot](case_04_rviz.png) |

The planner returned successful paths for all four cases. The existing validation helper checked the oriented 0.751 x 0.542 m footprint, with 0.02 m padding, along interpolated paths against the scene boxes. Coordinates are metres in the map frame. Start and goal arrows show requested headings.

These are global planning results with explicit virtual start poses. The robot was not driven or teleported to each start. This is not an MPPI execution demonstration or a SLAM mapping run. The occupancy map was generated from the same collision boxes used by Gazebo.

Sensing remains camera:=CPU ray casting, PointCloud2 -> camera_to_scan -> /scan, with IMU/EKF and both visible interfaces. Existing planner/controller parameters are unchanged. The isolated ROS domain is 94.

Files: world.sdf, map.pgm/map.yaml, the separate path_evidence.rviz view, per-case JSON paths, summary.json, actual PNG screenshots, raw PPM captures and session logs. No images were composited or annotated after capture: labels, paths and arrows were rendered in RViz.

Gazebo and RViz were left open on the combined overview. Stop this scene before starting another session in domain 94. capture_paths.py is a new one-off capture helper for this evidence folder; its --attach option was used to reconnect to the same running Gazebo session after correcting only the newly created RViz view.
