Visible GUI verification, 2026-10-28

Both Gazebo and RViz were enabled throughout these checks.
Default host OpenGL: llvmpipe (LLVM 20.1.2, 256 bits).
Scoped launch selection: D3D12 (NVIDIA GeForce GTX 1660 SUPER).
Actual Gazebo window captures are provided; navigation plots are not RViz screenshots.

interactive_navigation_summary.json: three paths/goals and an inserted obstacle.
maze_performance.json: short same-seed before/after wall-box-merging checks.

After validation, reopened a fresh random maze, seed 1866247602, with both
interfaces visible. Initialized AMCL at the known spawn (-22.5,22.5,0),
confirmed Nav2 active and the rendered scene. No motion goal was sent in this
interactive user launch. Close Gazebo to stop it before starting another launch.

Camera FOV/range, ray sampling and physics timestep were unchanged by the
GUI repair and maze optimization. Full real-time/full-maze navigation reliability
is not established by these short checks.
