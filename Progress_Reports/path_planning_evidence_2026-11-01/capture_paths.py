#!/usr/bin/env python3
"""Create a separate Gazebo scene and capture actual RViz Hybrid A* paths.

Only files in this new evidence directory are written. Existing launch files,
robot models, controller parameters and the progress report are read unchanged.
"""

import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as PathMessage
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


OUTPUT = Path(__file__).resolve().parent
WORKSPACE = OUTPUT.parents[1]
CASES = [
    ('Across the central obstacles', (0, 0, 0), (9, 0, 0)),
    ('Lower left to upper right', (0, -3, 0), (9, 3, math.pi / 2)),
    ('Upper corridor, heading west', (9, 3, math.pi), (0, 3, math.pi)),
    ('Lower right to upper left', (9, -3, math.pi), (0, 2.8, math.pi)),
]
COLORS = [(0.12, 0.92, 0.43), (0.15, 0.7, 1.0),
          (1.0, 0.65, 0.1), (0.93, 0.25, 0.8)]


def helper_module():
    spec = importlib.util.spec_from_file_location(
        'existing_validation_helpers', WORKSPACE /
        'src/robot_navigation/scripts/validate_holonomic_navigation.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_scene(helper):
    helper.BOXES[:] = [(-1.25, 0, .5, 9.5), (10.25, 0, .5, 9.5),
                       (4.5, -4.5, 11, .5), (4.5, 4.5, 11, .5),
                       (3, 0, 1, 2), (6, 2, 1.2, 1.2), (7, -2, 1, 1.2)]
    helper.make_world(OUTPUT)
    tree = ET.parse(OUTPUT / 'world.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'path_planning_gallery')
    for visual in world.findall("model/link/visual"):
        if visual.find('material') is None:
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = '.55 .58 .64 1'
            ET.SubElement(material, 'diffuse').text = '.55 .58 .64 1'
    tree.write(OUTPUT / 'world.sdf', encoding='unicode')


def marker(kind, number, x, y, z=.15, text='', color=(1., 1., 1.)):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns = 'path_evidence'
    m.id = number
    m.type = kind
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    m.pose.orientation.w = 1.
    m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.
    m.text = text
    return m


def text_marker(number, x, y, text, size=.26, color=(1., 1., 1.)):
    m = marker(Marker.TEXT_VIEW_FACING, number, x, y, text=text, color=color)
    m.scale.z = size
    return m


def endpoint_markers(start, goal, offset=0, prefix=''):
    result = []
    for i, (values, label, color) in enumerate([
            (start, prefix + 'START', (.2, .75, 1.)),
            (goal, prefix + 'GOAL', (1., .25, .6))]):
        x, y, angle = values
        arrow = marker(Marker.ARROW, offset + i * 3, x, y, color=color)
        arrow.pose.orientation.z = math.sin(angle / 2)
        arrow.pose.orientation.w = math.cos(angle / 2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .65, .15, .15
        result.append(arrow)
        result.append(text_marker(offset + i * 3 + 1, x, y + .4, label, .22, color))
    return result


def capture(label):
    windows = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    for name, pattern in [
            (label, r'(0x[0-9a-f]+) "[^"\n]*path_evidence\.rviz[^"\n]*RViz"'),
            ('gazebo_scene', r'(0x[0-9a-f]+) "Gazebo Sim"')]:
        if name == 'gazebo_scene' and (OUTPUT / 'gazebo_scene.png').exists():
            continue
        match = re.search(pattern, windows)
        if not match:
            raise RuntimeError('Visible window not found: ' + name)
        ppm = OUTPUT / (name + '.ppm')
        subprocess.run(['/tmp/flow_capture', match[1], str(ppm)], check=True)
        Image.open(ppm).save(OUTPUT / (name + '.png'))


def main():
    os.environ['ROS_DOMAIN_ID'] = '94'
    os.environ['GZ_PARTITION'] = 'flow_path_gallery_' + uuid.uuid4().hex
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['ROS_LOG_DIR'] = str(OUTPUT / 'ros_logs')
    helper = helper_module()
    attaching = '--attach' in sys.argv
    if not attaching:
        make_scene(helper)
    else:
        helper.BOXES[:] = [(-1.25, 0, .5, 9.5), (10.25, 0, .5, 9.5),
                           (4.5, -4.5, 11, .5), (4.5, 4.5, 11, .5),
                           (3, 0, 1, 2), (6, 2, 1.2, 1.2), (7, -2, 1, 1.2)]
    log = None
    if attaching:
        class ExistingSession:
            pid = 33030

            def poll(self):
                try:
                    os.kill(self.pid, 0)
                    return None
                except ProcessLookupError:
                    return 1

        process = ExistingSession()
    else:
        log = (OUTPUT / 'launch.log').open('w')
        process = subprocess.Popen([
        'ros2', 'launch', 'robot_navigation', 'navigation_launch.py',
        'world:=' + str(OUTPUT / 'world.sdf'), 'map:=' + str(OUTPUT / 'map.yaml'),
        'rviz_config:=' + str(OUTPUT / 'path_evidence.rviz'),
        'sensor:=camera', 'camera_backend:=cpu', 'camera_fps:=10',
        'gui:=true', 'use_rviz:=true', 'use_imu:=true',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    print('OUTPUT', OUTPUT, 'LAUNCH_PID', process.pid, flush=True)
    rclpy.init()
    node = rclpy.create_node('path_screenshot_session', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(PathMessage, '/evidence/path', qos)
    markers_pub = node.create_publisher(MarkerArray, '/evidence/markers', qos)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def until(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(node, timeout_sec=.05)
            if process.poll() is not None:
                raise RuntimeError('Gazebo/launch exited; inspect launch.log')
        if not predicate():
            raise TimeoutError('Timed out waiting for planning/display readiness')

    def pause(seconds):
        end = time.monotonic() + seconds
        until(lambda: time.monotonic() >= end, seconds + 1)

    def active(name):
        client = node.create_client(GetState, '/' + name + '/get_state')
        until(client.service_is_ready, 60)
        for _ in range(150):
            f = client.call_async(GetState.Request())
            until(f.done, 5)
            if f.result().current_state.id == 3:
                return
            pause(.2)
        raise RuntimeError('Node did not activate: ' + name)

    active('amcl')
    until(lambda: initial_pub.get_subscription_count() > 0, 10)
    initial = PoseWithCovarianceStamped()
    initial.header.frame_id = 'map'
    initial.pose.pose.orientation.w = 1.
    initial.pose.covariance[0] = initial.pose.covariance[7] = .01
    initial.pose.covariance[35] = .01
    for _ in range(20):
        if not attaching:
            initial_pub.publish(initial)
        pause(1)
        if buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
            break
    active('bt_navigator')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    until(planner.server_is_ready, 15)
    until(lambda: path_pub.get_subscription_count() > 0, 20)
    until(lambda: markers_pub.get_subscription_count() > 0, 20)
    records = []
    for index, (name, start, goal) in enumerate(CASES, 1):
        request = ComputePathToPose.Goal()
        request.start, request.goal = helper.pose(start), helper.pose(goal)
        request.use_start = True
        request.planner_id = 'GridBased'
        sent = planner.send_goal_async(request)
        until(sent.done, 10)
        handle = sent.result()
        assert handle.accepted
        future = handle.get_result_async()
        until(future.done, 60)
        result = future.result()
        assert result.status == 4 and result.result.error_code == 0, str(result)
        path = result.result.path
        points = [[p.pose.position.x, p.pose.position.y, helper.yaw(p.pose.orientation)]
                  for p in path.poses]
        helper.check_path(points)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
        record = {'case': index, 'name': name, 'start': start, 'goal': goal,
                  'length_m': length, 'poses': points,
                  'planning_seconds': result.result.planning_time.sec
                  + result.result.planning_time.nanosec * 1e-9,
                  'footprint_clearance_check': 'passed',
                  'screenshot': f'case_{index:02d}_rviz.png'}
        (OUTPUT / f'case_{index:02d}_path.json').write_text(json.dumps(record, indent=2) + '\n')
        records.append(record)
        clear = Marker(action=Marker.DELETEALL)
        labels = [clear, text_marker(100, 4.5, 5.35, f'SMAC HYBRID A* | CASE {index}: {name}', .30),
                  text_marker(101, 4.5, 4.88,
                              f'S ({start[0]}, {start[1]}, {round(math.degrees(start[2]))} deg)'
                              f'   ->   G ({goal[0]}, {goal[1]}, {round(math.degrees(goal[2]))} deg)', .25),
                  text_marker(102, 4.5, -5.2,
                              f'{length:.2f} m | {len(points)} poses | footprint clearance checked | planning only', .24)]
        labels += endpoint_markers(start, goal, 200)
        path_pub.publish(path)
        markers_pub.publish(MarkerArray(markers=labels))
        pause(3)
        capture(f'case_{index:02d}_rviz')
        print('CASE_PASS', index, name, f'{length:.2f}m', len(points), flush=True)
    overview = [Marker(action=Marker.DELETEALL),
                text_marker(100, 4.5, 5.35, 'SMAC HYBRID A* | FOUR START / GOAL PAIRS', .32),
                text_marker(101, 4.5, -6.35, 'Computed global paths in the same Gazebo obstacle map | planning only', .23)]
    for i, record in enumerate(records):
        line = marker(Marker.LINE_STRIP, 300 + i, 0, 0, z=.1, color=COLORS[i])
        line.scale.x = .055
        line.points = [Point(x=p[0], y=p[1], z=0.) for p in record['poses']]
        overview += [line, text_marker(400 + i, 4.5, -5.1 - i * .29,
                                      f"{i+1}: {record['name']} ({record['length_m']:.2f} m)", .22, COLORS[i])]
        overview += endpoint_markers(record['start'], record['goal'], 500 + i * 10, f'{i+1} ')
    empty_path = PathMessage()
    empty_path.header.frame_id = 'map'
    path_pub.publish(empty_path)
    markers_pub.publish(MarkerArray(markers=overview))
    pause(3)
    capture('all_paths_rviz')
    summary = {'planner': 'nav2_smac_planner::SmacPlannerHybrid', 'motion_model': 'DUBIN',
               'domain': 94, 'world': 'path_planning_gallery', 'cases': records,
               'note': 'Actual RViz window screenshots. Explicit virtual planning starts; '
                       'the robot is not teleported or driven to these starts. '
                       'Occupancy map is generated from the same boxes as Gazebo, not SLAM.'}
    (OUTPUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('SCREENSHOTS_COMPLETE; leaving Gazebo/RViz and path publishers running', flush=True)
    while process.poll() is None:
        rclpy.spin_once(node, timeout_sec=.25)
    node.destroy_node()
    rclpy.shutdown()
    if log is not None:
        log.close()


if __name__ == '__main__':
    main()
