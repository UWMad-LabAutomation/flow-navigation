#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""
Exercise Hybrid A* and Omni MPPI in an isolated, reproducible Gazebo room.

Ground truth is recorded for verification only, never published into Nav2 TF.
The normal AMCL, CPU camera, scan converter and collision monitor stay active.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


BOXES = [(-1.25, 0, 0.5, 7), (8.25, 0, 0.5, 7),
         (3.5, -3.25, 9, 0.5), (3.5, 3.25, 9, 0.5),
         (2.5, 0, 0.8, 1.0), (5, -1, 0.6, 0.8)]
PLAN_PAIRS = [((0, 0, 0), (4, 0, 0)),
              ((1, 1, 0), (4, -1, math.pi / 2)),
              ((4, 1.5, 0), (1, 1.5, math.pi))]
GOALS = [(1, 1, 0), (4, 0, 0), (4, 1.4, 0)]


def yaw(q):
    """Extract planar heading from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def pose(values):
    """Create an unstamped map pose."""
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x, msg.pose.position.y = map(float, values[:2])
    msg.pose.orientation.z = math.sin(values[2] / 2)
    msg.pose.orientation.w = math.cos(values[2] / 2)
    return msg


def collides(x, y, angle, padding=0.02):
    """Check the oriented FLOW footprint against the known test rectangles."""
    c, s = math.cos(angle), math.sin(angle)
    hx, hy = 0.3755 + padding, 0.271 + padding
    axes = [(1, 0), (0, 1), (c, s), (-s, c)]
    for bx, by, width, height in BOXES:
        separated = False
        for ax, ay in axes:
            robot_r = hx * abs(ax * c + ay * s) + hy * abs(-ax * s + ay * c)
            box_r = width / 2 * abs(ax) + height / 2 * abs(ay)
            if abs((x - bx) * ax + (y - by) * ay) > robot_r + box_r:
                separated = True
                break
        if not separated:
            return True
    return False


def make_world(output):
    """Write matching collision geometry and a 5 cm occupancy map."""
    source = Path(get_package_share_directory('robot_simulation')) / 'worlds/flow_base.sdf'
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    world.set('name', 'holonomic_validation')
    for index, (x, y, width, height) in enumerate(BOXES):
        model = ET.SubElement(world, 'model', name=f'validation_box_{index}')
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{x} {y} 0.4 0 0 0'
        link = ET.SubElement(model, 'link', name='body')
        for kind in ['collision', 'visual']:
            item = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(item, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = f'{width} {height} 0.8'
    tree.write(output / 'world.sdf', encoding='unicode')
    xmin = min(x - width / 2 for x, y, width, height in BOXES)
    xmax = max(x + width / 2 for x, y, width, height in BOXES)
    ymin = min(y - height / 2 for x, y, width, height in BOXES)
    ymax = max(y + height / 2 for x, y, width, height in BOXES)
    columns, rows = round((xmax - xmin) / 0.05), round((ymax - ymin) / 0.05)
    xx = xmin + (np.arange(columns) + 0.5) * 0.05
    yy = ymin + (np.arange(rows) + 0.5) * 0.05
    x, y = np.meshgrid(xx, yy)
    occupied = np.zeros(x.shape, dtype=bool)
    for bx, by, width, height in BOXES:
        occupied |= (abs(x - bx) <= width / 2) & (abs(y - by) <= height / 2)
    pixels = np.where(occupied, 0, 254).astype(np.uint8)[::-1]
    (output / 'map.pgm').write_bytes(
        f'P5\n{columns} {rows}\n255\n'.encode() + pixels.tobytes())
    (output / 'map.yaml').write_text(
        f'image: map.pgm\nresolution: 0.05\norigin: [{xmin}, {ymin}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def check_path(points):
    """Check footprint clearance between poses, including changing heading."""
    for a, b in zip(points, points[1:]):
        angle = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        steps = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.025),
                    math.ceil(abs(angle) / 0.05))
        for f in np.linspace(0, 1, steps + 1):
            assert not collides(a[0] + f * (b[0] - a[0]),
                                a[1] + f * (b[1] - a[1]), a[2] + f * angle), \
                'Planned footprint intersects obstacle'


def motion_metrics(samples):
    """Measure body-forward travel and startup motion inside the camera's HFOV."""
    distance = forward = side = in_view = startup_distance = startup_forward = 0.0
    startup_in_view = 0.0
    for a, b in zip(samples, samples[1:]):
        dt = b[0] - a[0]
        if not 0 < dt < 0.5:
            continue
        vx, vy = b[4:6]
        step = math.hypot(vx, vy) * dt
        distance += step
        forward += max(vx, 0) * dt
        side += abs(vy) * dt
        visible = vx > 0 and abs(math.atan2(vy, vx)) <= math.radians(43.5)
        if visible:
            in_view += step
        if startup_distance < 0.5:
            fraction = min(1.0, (0.5 - startup_distance) / step) if step else 0
            startup_distance += step * fraction
            startup_forward += max(vx, 0) * dt * fraction
            if visible:
                startup_in_view += step * fraction
    return {
        'travel_m': distance, 'body_lateral_travel_m': side,
        'forward_distance_fraction': forward / max(distance, 1e-9),
        'travel_inside_forward_hfov_fraction': in_view / max(distance, 1e-9),
        'first_0_5m_forward_fraction': startup_forward / max(startup_distance, 1e-9),
        'first_0_5m_inside_forward_hfov_fraction': startup_in_view / max(startup_distance, 1e-9),
    }


def pose_accuracy(estimated, truth):
    """Compare map/EKF poses with ground truth at the same simulation times."""
    reference = np.array(list({p[0]: p for p in truth}.values()))
    reference = reference[np.argsort(reference[:, 0])]
    reference[:, 3] = np.unwrap(reference[:, 3])
    values = np.array(estimated)[:, :4]
    values = values[(values[:, 0] >= reference[0, 0]) & (values[:, 0] <= reference[-1, 0])]
    assert len(values) > 10, 'Insufficient synchronized pose samples'
    actual = np.column_stack([np.interp(values[:, 0], reference[:, 0], reference[:, i])
                              for i in (1, 2, 3)])
    position = np.linalg.norm(values[:, 1:3] - actual[:, :2], axis=1)
    angle = np.arctan2(np.sin(values[:, 3] - actual[:, 2]),
                       np.cos(values[:, 3] - actual[:, 2]))
    return {'position_rmse_m': float(np.sqrt(np.mean(position ** 2))),
            'position_max_m': float(np.max(position)),
            'heading_rmse_deg': float(np.degrees(np.sqrt(np.mean(angle ** 2)))),
            'heading_max_deg': float(np.degrees(np.max(np.abs(angle))))}


def run(output, domain, dynamic_obstacle=False, clear_start=False):
    """Launch, plan, navigate, record evidence, and stop only this test's processes."""
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['GZ_PARTITION'] = 'flow_validation_' + uuid.uuid4().hex
    os.environ['ROS_LOG_DIR'] = str(output / 'ros')
    os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    if clear_start:
        # Keep front walls outside the 1.5 m avoidance zone at each startup.
        BOXES[:] = [(-1.25, 0, 0.5, 11), (8.25, 0, 0.5, 11),
                    (3.5, -5.25, 9, 0.5), (3.5, 5.25, 9, 0.5)]
        GOALS[:] = [(1, 1.8, math.pi / 2), (5.0, 1.8, 0)]
    make_world(output)
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'robot_navigation', 'navigation_launch.py',
        'world:=' + str(output / 'world.sdf'), 'map:=' + str(output / 'map.yaml'),
        'gui:=true', 'use_rviz:=true', 'camera_backend:=cpu', 'camera_fps:=10',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('holonomic_validation')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data = {'plans': [], 'goals': [], 'commands': [], 'odometry': [], 'truth': [],
            'localization': [], 'map_poses': [], 'truth_stamped': [],
            'requested_goals': list(GOALS), 'static_boxes': list(BOXES), 'clock': []}
    last_sample = [0.0]
    scans = []

    def on_odom(msg):
        data['odometry'].append([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])

    subscriptions = [
        node.create_subscription(Clock, '/clock', lambda m: data['clock'].append([
            time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9]), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data),
        node.create_subscription(Twist, '/cmd_vel', lambda m: data['commands'].append(
            [m.linear.x, m.linear.y, m.angular.z]), qos_profile_sensor_data),
    ]
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GazeboNode
    gazebo = GazeboNode()
    seen_models = set()

    def ground_truth(msg):
        for p in msg.pose:
            seen_models.add(p.name)
            if p.name == 'flow_base':
                data['truth'].append([p.position.x, p.position.y, yaw(p.orientation)])
                data['truth_stamped'].append([
                    msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                    p.position.x, p.position.y, yaw(p.orientation)])

    gazebo.subscribe(Pose_V, '/world/holonomic_validation/pose/info', ground_truth)

    def until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            rclpy.spin_once(node, timeout_sec=0.02)
            if data['truth'] and time.monotonic() - last_sample[0] > 0.2:
                try:
                    stamped_tf = buffer.lookup_transform(
                        'map', 'base_footprint', rclpy.time.Time())
                    tf = stamped_tf.transform
                    data['map_poses'].append([
                        stamped_tf.header.stamp.sec + stamped_tf.header.stamp.nanosec * 1e-9,
                        tf.translation.x, tf.translation.y, yaw(tf.rotation)])
                    actual = data['truth'][-1]
                    angle_error = math.atan2(math.sin(yaw(tf.rotation) - actual[2]),
                                             math.cos(yaw(tf.rotation) - actual[2]))
                    data['localization'].append([
                        tf.translation.x, tf.translation.y, yaw(tf.rotation),
                        math.hypot(tf.translation.x - actual[0],
                                   tf.translation.y - actual[1]), angle_error])
                    last_sample[0] = time.monotonic()
                except Exception:
                    pass
            if process.poll() is not None:
                raise RuntimeError('Launch exited; see launch.log')
        if not predicate():
            raise TimeoutError('Timed out; see launch.log')

    def active(name):
        client = node.create_client(GetState, '/' + name + '/get_state')
        until(client.service_is_ready, 60)
        for _ in range(150):
            future = client.call_async(GetState.Request())
            until(future.done, 5)
            if future.result().current_state.id == 3:
                return
            end = time.monotonic() + 0.2
            until(lambda: time.monotonic() >= end, 1)
        raise TimeoutError(name + ' did not activate')

    def action(client, goal, timeout):
        until(client.server_is_ready, 10)
        sent = client.send_goal_async(goal)
        until(sent.done, 10)
        handle = sent.result()
        assert handle.accepted, 'Goal rejected'
        future = handle.get_result_async()
        try:
            until(future.done, timeout)
        except TimeoutError:
            cancelled = handle.cancel_goal_async()
            until(cancelled.done, 5)
            raise
        result = future.result()
        assert result.status == 4 and result.result.error_code == 0, str(result)
        return result.result

    try:
        active('amcl')
        until(lambda: bool(scans), 30)
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.orientation.w = 1.0
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01
        initial.pose.covariance[35] = 0.01
        until(lambda: publisher.get_subscription_count() > 0, 10)
        # Discovery counts can precede a usable DDS connection. Retry only at
        # startup, before any goal, until AMCL actually supplies map -> odom.
        for _ in range(15):
            publisher.publish(initial)
            deadline = time.monotonic() + 1
            until(lambda: buffer.can_transform('map', 'base_footprint', rclpy.time.Time())
                  or time.monotonic() >= deadline, 2)
            if buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                break
        else:
            raise TimeoutError('AMCL did not accept the initial pose')
        active('bt_navigator')
        print('NAV2_ACTIVE', flush=True)
        planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        for start, goal in PLAN_PAIRS:
            request = ComputePathToPose.Goal()
            request.start, request.goal = pose(start), pose(goal)
            request.use_start = True
            request.planner_id = 'GridBased'
            begin = time.monotonic()
            result = action(planner, request, 30)
            points = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                      for p in result.path.poses]
            assert len(points) > 2
            assert math.hypot(points[-1][0] - goal[0], points[-1][1] - goal[1]) < 0.26
            check_path(points)
            heading_error = points[-1][2] - goal[2]
            assert abs(math.atan2(math.sin(heading_error), math.cos(heading_error))) < 0.1
            data['plans'].append({'start': start, 'goal': goal, 'poses': points,
                                  'wall_seconds': time.monotonic() - begin})
            print('PLAN_PASS', start, goal, len(points), flush=True)
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        for index, goal in enumerate(GOALS):
            if dynamic_obstacle and index == 1:
                from google.protobuf.text_format import MessageToString
                from gz.msgs10.entity_factory_pb2 import EntityFactory
                obstacle = (2.8, 1.4, 0.5, 0.5)
                sdf = ('<sdf version="1.9"><model name="inserted_obstacle">'
                       '<static>true</static><pose>2.8 1.4 0.4 0 0 0</pose>'
                       '<link name="body"><collision name="body"><geometry>'
                       '<box><size>0.5 0.5 0.8</size></box></geometry></collision>'
                       '<visual name="body"><geometry><box><size>0.5 0.5 0.8</size>'
                       '</box></geometry></visual></link></model></sdf>')
                # A separate client avoids blocking Python pose callbacks while
                # the transport request waits for a response.
                response = subprocess.run([
                    'gz', 'service', '-s', '/world/holonomic_validation/create',
                    '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '10000', '--req', MessageToString(EntityFactory(sdf=sdf)),
                ], capture_output=True, text=True, timeout=15)
                assert response.returncode == 0, response.stderr
                until(lambda: 'inserted_obstacle' in seen_models, 15)
                BOXES.append(obstacle)
                data['inserted_obstacle'] = obstacle
                print('INSERTED_UNMAPPED_OBSTACLE', obstacle, flush=True)
            request = NavigateToPose.Goal()
            request.pose = pose(goal)
            begin = time.monotonic()
            odom_start = len(data['odometry'])
            action(navigator, request, 120)
            actual = data['truth'][-1]
            error = math.hypot(actual[0] - goal[0], actual[1] - goal[1])
            tf = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            localization_error = math.hypot(tf.translation.x - actual[0],
                                            tf.translation.y - actual[1])
            data['goals'].append({'goal': goal, 'actual': actual, 'position_error': error,
                                  'localization_error': localization_error,
                                  'motion': motion_metrics(data['odometry'][odom_start:]),
                                  'wall_seconds': time.monotonic() - begin})
            assert error < 0.4, 'Reported success disagrees with Gazebo position'
            print('NAV_PASS', data['goals'][-1], flush=True)
            if clear_start:
                metrics = data['goals'][-1]['motion']
                assert metrics['forward_distance_fraction'] > 0.9, metrics
                assert metrics['first_0_5m_inside_forward_hfov_fraction'] > 0.9, metrics
        commands = data['commands']
        assert commands and max(math.hypot(*v[:2]) for v in commands) <= 0.630001
        if not clear_start:
            assert max(abs(v[1]) for v in commands) > 0.05, 'No lateral avoidance capability'
        lateral = sum(abs(b[5]) * (b[0] - a[0])
                      for a, b in zip(data['odometry'], data['odometry'][1:]))
        if not clear_start:
            assert lateral > 0.2, 'No lateral avoidance movement'
        assert not any(collides(*p, padding=0) for p in data['truth']), 'Robot collision'
        data['summary'] = {
            'max_command_speed': max(math.hypot(*v[:2]) for v in commands),
            'max_lateral_command': max(abs(v[1]) for v in commands),
            'lateral_travel': lateral,
            'scan_publishers': [p.node_name for p in node.get_publishers_info_by_topic('/scan')],
            'ekf_accuracy': pose_accuracy(data['odometry'], data['truth_stamped']),
            'map_localization_accuracy': pose_accuracy(data['map_poses'], data['truth_stamped']),
            'real_time_factor': ((data['clock'][-1][1] - data['clock'][0][1])
                                 / (data['clock'][-1][0] - data['clock'][0][0])),
            'gazebo_gui': True, 'rviz': True,
        }
        assert data['summary']['scan_publishers'] == ['camera_to_scan']
        print('PASS', json.dumps(data['summary']), flush=True)
    finally:
        # Deactivate navigation while clock/TF still exist. Simultaneous SIGINT
        # exposed an upstream collision-monitor teardown race after goal success.
        for name in ['lifecycle_manager_navigation', 'lifecycle_manager_localization']:
            try:
                client = node.create_client(ManageLifecycleNodes, '/' + name + '/manage_nodes')
                if client.service_is_ready():
                    request = ManageLifecycleNodes.Request()
                    request.command = ManageLifecycleNodes.Request.SHUTDOWN
                    future = client.call_async(request)
                    deadline = time.monotonic() + 5
                    while not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                pass
        del subscriptions, listener
        node.destroy_node()
        rclpy.shutdown()
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
            if process.poll() is not None:
                break
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        log.close()
        (output / 'results.json').write_text(json.dumps(data, indent=2))
        plot(output, data)


def plot(output, data):
    """Save a plot of computed paths and measured Gazebo travel."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax in axes:
        for x, y, width, height in data.get('static_boxes', BOXES[:6]):
            ax.add_patch(Rectangle((x - width / 2, y - height / 2), width, height,
                                   color='0.35'))
        boxes = data.get('static_boxes', BOXES[:6])
        ax.set(xlim=(min(b[0] - b[2] / 2 for b in boxes),
                     max(b[0] + b[2] / 2 for b in boxes)),
               ylim=(min(b[1] - b[3] / 2 for b in boxes),
                     max(b[1] + b[3] / 2 for b in boxes)), xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal')
        ax.grid(alpha=0.2)
    if 'inserted_obstacle' in data:
        x, y, width, height = data['inserted_obstacle']
        axes[1].add_patch(Rectangle((x - width / 2, y - height / 2), width, height,
                                    color='tab:orange', label='Inserted after planning'))
    for index, path in enumerate(data['plans']):
        p = np.array(path['poses'])
        axes[0].plot(p[:, 0], p[:, 1], label=f'Plan {index + 1}')
        axes[0].plot(*p[0, :2], 'o')
        axes[0].plot(*p[-1, :2], 'x')
    if data['truth']:
        p = np.array(data['truth'])
        axes[1].plot(p[:, 0], p[:, 1], color='tab:blue', label='Gazebo measured motion')
    for index, goal in enumerate(data.get('requested_goals', GOALS)):
        axes[1].plot(*goal[:2], 'rx')
        axes[1].annotate(str(index + 1), goal[:2])
    axes[0].set_title('Smac Hybrid A* paths')
    axes[1].set_title('Omni MPPI navigation')
    for ax in axes:
        if ax.lines:
            ax.legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(output / 'paths_and_navigation.png', dpi=160)
    plt.close(fig)


def main():
    """Parse output location and ROS isolation domain."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=72)
    parser.add_argument('--dynamic-obstacle', action='store_true',
                        help='Insert an unmapped obstacle after the first navigation goal.')
    parser.add_argument('--clear-start', action='store_true',
                        help='Check forward-facing startup and turns in an unobstructed room.')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    print('ARTIFACTS', args.output, flush=True)
    if args.clear_start and args.dynamic_obstacle:
        parser.error('--clear-start and --dynamic-obstacle select different courses')
    run(args.output, args.domain, args.dynamic_obstacle, args.clear_start)


if __name__ == '__main__':
    main()
