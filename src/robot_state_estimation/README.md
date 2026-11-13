# FLOW wheel/IMU state estimation

`robot_localization` fuses wheel body velocities (vx, vy, yaw rate) and the
Gemini 335L gyro into `/odom` and `odom -> base_footprint`. AMCL continues to produce
`map -> odom` from the map and camera-derived `/scan`. Fusion runs on the CPU.
An IMU improves short-term motion estimation; it cannot determine absolute map
position or eliminate map-matching errors by itself. Gyro bias still accumulates.

The filter uses all gyro axes and the sensor-to-base TF to account for the tilted
camera. It uses no IMU orientation or linear acceleration. The Gemini 335L has no
absolute yaw reference, and elevator/arm acceleration must not move the base
estimate. Wheel vy is measured, not constrained to zero. Sensor covariances must
be calibrated on hardware; simulation fallback values are not hardware calibration.

Simulation launches this filter automatically with `use_imu:=true` (default).
The Gazebo IMU uses the camera mount pose and does not need rendering. Gazebo's
raw odometry TF is not bridged while the EKF owns that transform. Use
`use_imu:=false` for the previous wheel-only simulation pipeline.

For future hardware integration, install the Orbbec ROS 2 driver and enable
the Gemini 335L motion streams. This example is a wiring plan; a complete
physical bringup launch and hardware validation are not present here:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_gyro:=true enable_accel:=true enable_sync_output_accel_gyro:=true \
  enable_point_cloud:=true
```

With the driver's default camera namespace, the combined topic is
`/camera/gyro_accel/sample`, with frame `camera_accel_gyro_optical_frame`.
The point cloud is `/camera/depth/points`. Start state estimation against the
real base's wheel topic and select the combined IMU input:

```bash
ros2 launch robot_state_estimation state_estimation.launch.py \
  use_sim_time:=false wheel_odom_topic:=/wheel/odometry \
  imu_topic:=/camera/gyro_accel/sample
```

The actual base must publish meaningful wheel twist covariances and stop
publishing `odom -> base_footprint` itself. Its odometry child frame must be
connected to `base_footprint`. The robot description must supply the measured
base-to-camera mount, and the Orbbec driver supplies camera-to-gyro TF.
Disable nominal simulated camera frames when using the driver's calibrated
frames (`camera_nominal_depth_frames:=false` in the Xacro). Camera elevator
motion needs an updated mount transform on hardware. Physical camera/robot
fusion has not been tested in this WSL checkout.

The [Orbbec topic reference](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/topics.html)
and [launch parameters](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html)
document these inputs. Check the installed driver's topics and calibrated
frames before connecting hardware. Simulation retains `/camera/imu` and its
nominal camera frame; changing the camera profile does not calibrate IMU noise.

From the sourced workspace, reproduce the isolated checks with new output paths:

```bash
python3 src/robot_state_estimation/scripts/check_imu_fusion.py \
  --output /tmp/flow_imu_bias_check
python3 src/robot_state_estimation/scripts/check_simulated_imu.py \
  --output /tmp/flow_imu_physics_check
```

The first test deliberately biases wheel yaw rate and confirms gyro correction
and lateral velocity. The second drives only its own isolated Gazebo world,
checks the actual camera-to-base rotation, odometry/TF ownership, and sensor
rates. The default simulation requested 100 Hz IMU and measured about 95 Hz
of simulation time in this run; filtered odometry measured 50 Hz. The standard
navigation course also passed three goals with a newly inserted obstacle.
Evidence is in `Progress_Reports/imu_localization_2026-10-28/`. None of these
checks establish hardware bias/noise calibration or full-maze reliability.
The recorded results used the previous geometry and D435i profile, before the
November 9, 2026 update; they do not validate the revised rail/camera mount.


The accuracy check now runs 24 simulated seconds of straight/curved travel,
a curved sidestep and a stop. It interpolates Gazebo truth to each IMU/EKF
timestamp and saves raw `accuracy_samples.npz`. In the recorded run, EKF
maximum position/heading errors were 9.03 mm / 0.116 degrees and gyro-rate RMSE
was 0.001019 rad/s. These describe the configured simulation noise model.
Global AMCL map errors were larger (up to 0.146 m / 3.25 degrees in the
inserted-obstacle course). See `Progress_Reports/forward_navigation_2026-10-28/`.
