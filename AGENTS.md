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

## Navigation Architecture
- Use Nav2 as the navigation framework.
- SLAM provides map data and map -> odom.
- Base/state-estimation layer provides odom -> base_link.
- Nav2 outputs geometry_msgs/Twist commands.
- A ROS 2 bridge will convert /cmd_vel into I2RT FlowBaseClient commands.
- Navigation owns vx, vy, and yaw rate.
- VLA/manipulation owns the rail and arms.
- Keep navigation CPU-oriented where practical so GPU resources remain available for VLA and SLAM.

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
Work on Step 1 only unless explicitly told to move on.