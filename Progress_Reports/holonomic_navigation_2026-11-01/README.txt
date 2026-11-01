MPPI Omni / Hybrid A* visible simulation demonstration, 2026-11-01

Configuration: src/robot_navigation/config/nav2_flow_params.yaml.
No planner/controller/sensor parameter changes were needed for this check.
CPU geometric camera, 80x45 rays, 87x58 deg, 0.3-3m optical depth, 2m mount,
auto pitch; IMU/EKF enabled; camera_fps:=10 simulation Hz. No rendered depth
sensor or LiDAR; /scan published only by camera_to_scan.

All three planned paths and all three navigation goals passed with an inserted
unmapped obstacle. The checker verified no robot footprint overlap with test
boxes using Gazebo truth and a total translation speed <=0.63m/s.
Peak sideways command 0.1059m/s; integrated lateral travel 0.488m.
Goal position errors: 0.0665m, 0.0535m, 0.1915m. Whole-run RTF 0.768.

Gazebo/RViz screenshots are actual window captures. paths_and_navigation.png
is a plot of computed paths and measured motion. Raw results and failed first
attempt logs are in /tmp/flow_holonomic_2026-11-01_run{1,2} for this session.
See validation_summary.json for outcomes and limitations.

Reproduce in a new output directory:
source /opt/ros/jazzy/setup.bash
source /home/zach/robot_ws/install/setup.bash
python3 src/robot_navigation/scripts/validate_holonomic_navigation.py --output /tmp/flow_mppi_check_new --dynamic-obstacle --domain 93

After the check, a fresh random maze (seed 3905520988) was launched on the
normal ROS domain with Gazebo and RViz visible. AMCL was initialized at the
known spawn (-22.5,22.5,0); no motion goal was sent in that interactive launch.
interactive_live_controller.json records the running Omni MPPI plugin, limits
and horizon settings from that launch. Close Gazebo before starting another.
