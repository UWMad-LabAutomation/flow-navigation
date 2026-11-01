#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Exercise CPU-camera geometry, occlusion, clipping, and moving collision objects."""

from collections import deque
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node
from robot_simulation.sensor_configuration import configure_sensor_world, POINTS_TOPIC
import xacro


def box(name, x, y, z, sx, sy, sz):
    """Return a static collision-only box, deliberately invisible to rendering."""
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{x} {y} {z} 0 0 0</pose><link name="body">'
            '<collision name="box"><geometry><box>'
            f'<size>{sx} {sy} {sz}</size></box></geometry></collision></link></model>')


def point(cloud, row, col):
    """Read XYZ from one organized point-cloud sample."""
    fields = {field.name: field.offset for field in cloud.field}
    base = row * cloud.row_step + col * cloud.point_step
    endian = '>' if cloud.is_bigendian else '<'
    return [struct.unpack_from(endian + 'f', cloud.data, base + fields[a])[0]
            for a in ('x', 'y', 'z')]


def check_floor(output, simulation, description):
    """Reject false raised-floor hits with the default tilted, dynamic robot."""
    robot = xacro.process_file(str(description / 'urdf/flow_base.urdf.xacro')).toxml()
    pitch = float(ET.fromstring(robot).find(
        "joint[@name='base_link_to_camera_link']/origin").get('rpy').split()[1])
    world = configure_sensor_world(simulation / 'worlds/flow_base.sdf', robot, 'camera')
    received = []
    ready = threading.Event()
    node = Node()

    def receive(message):
        received.append(message)
        if len(received) >= 100:
            ready.set()

    node.subscribe(PointCloudPacked, POINTS_TOPIC, receive)
    with (output / 'floor.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-r', '-s', str(world)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert ready.wait(timeout=25), f'Missing floor frames: {output / "floor.log"}'
            max_height_error = 0.0
            hits = 0
            for cloud in list(received):
                for row in range(cloud.height):
                    for col in range(cloud.width):
                        x, _, z = point(cloud, row, col)
                        if math.isfinite(x):
                            ground_z = 2.0 - math.sin(pitch) * x + math.cos(pitch) * z
                            max_height_error = max(max_height_error, abs(ground_z))
                            hits += 1
            assert hits > 100
            assert max_height_error < .005, max_height_error
            print(f'PASS: tilted-camera floor height error <= {max_height_error:.6f} m; '
                  'no floor points enter the obstacle height band', flush=True)
        finally:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if process.poll() is not None:
                    break
                os.killpg(process.pid, sig)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    continue


def main():
    """Run in an isolated partition and stop only the process group we start."""
    output = Path(tempfile.mkdtemp(prefix='flow_cpu_camera_check_'))
    simulation = Path(get_package_share_directory('robot_simulation'))
    description = Path(get_package_share_directory('robot_description'))
    world = configure_sensor_world(
        simulation / 'worlds/flow_base.sdf',
        xacro.process_file(str(description / 'urdf/flow_base.urdf.xacro'),
                           mappings={'camera_pitch': '0'}).toxml(), 'camera')
    tree = ET.parse(world)
    # Hold the robot still so ray geometry has an exact analytic reference.
    model = tree.find("world/model[@name='flow_base']")
    ET.SubElement(model, 'static').text = 'true'
    tree.find('world').append(ET.fromstring(box('reference_wall', 2.1, 0, 2, .2, 6, 6)))
    tree.write(world)

    os.environ['GZ_PARTITION'] = f'flow_cpu_check_{os.getpid()}'
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join([
        str(Path(get_package_prefix('robot_cpu_camera')) / 'lib'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ])
    # This check must work with neither a display nor an EGL rendering backend.
    os.environ.pop('DISPLAY', None)
    os.environ.pop('WAYLAND_DISPLAY', None)
    node = Node()
    condition = threading.Condition()
    received = deque(maxlen=180)

    def receive(message):
        with condition:
            received.append((time.monotonic(), message))
            condition.notify_all()

    def wait_for(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        with condition:
            while time.monotonic() < deadline:
                if received and predicate(received[-1][1]):
                    return received[-1][1]
                condition.wait(timeout=.2)
        raise AssertionError(f'Camera condition timed out; log: {output / "gazebo.log"}')

    def move(name, x, y, z):
        request = Pose(name=name)
        request.position.x, request.position.y, request.position.z = x, y, z
        request.orientation.w = 1
        ok, response = node.request('/world/flow_base_world/set_pose', request,
                                    Pose, Boolean, 3000)
        assert ok and response.data, f'Could not move {name}'

    node.subscribe(PointCloudPacked, POINTS_TOPIC, receive)
    with (output / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-r', '-s', str(world)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            cloud = wait_for(lambda c: math.isfinite(point(c, 22, 40)[0]), timeout=30)
            assert (cloud.width, cloud.height) == (80, 45)
            assert not cloud.is_dense
            assert any(d.key == 'frame_id' and list(d.value) == ['camera_depth_frame']
                       for d in cloud.header.data)
            fx = 80 / (2 * math.tan(math.radians(87) / 2))
            fy = 45 / (2 * math.tan(math.radians(58) / 2))
            for row, col in [(22, 40), (22, 0), (22, 79), (0, 40), (44, 40)]:
                expected = [2, 2 * (40 - col - .5) / fx, 2 * (22.5 - row - .5) / fy]
                actual = point(cloud, row, col)
                assert all(abs(a - b) < .005 for a, b in zip(actual, expected)), (
                    row, col, actual, expected)
            print('PASS: collision-only wall, 87x58 FOV, independent intrinsics and XYZ frame',
                  flush=True)

            # Insert after startup; the closest object must hide the far wall.
            request = EntityFactory(sdf='<sdf version="1.9">' +
                                    box('occluder', 1.1, 0, 2, .2, .8, .8) + '</sdf>')
            ok, response = node.request('/world/flow_base_world/create', request,
                                        EntityFactory, Boolean, 3000)
            assert ok and response.data
            wait_for(lambda c: abs(point(c, 22, 40)[0] - 1) < .005)
            print('PASS: newly inserted collision object occludes background', flush=True)
            move('occluder', 1.1, 2, 2)
            wait_for(lambda c: abs(point(c, 22, 40)[0] - 2) < .005)
            print('PASS: moving obstacle exposes background', flush=True)

            # Too-near geometry still occludes; it must not reveal the far wall.
            move('occluder', .2, 0, 2)
            wait_for(lambda c: all(math.isnan(v) for v in point(c, 22, 40)))
            print('PASS: near clipping preserves occlusion and reports unknown', flush=True)
            move('occluder', 1.1, 2, 2)
            move('reference_wall', 3.3, 0, 2)
            wait_for(lambda c: all(math.isnan(v) for v in point(c, 22, 40)))
            # Ensure this is not the old near-occluder frame.
            move('reference_wall', 2.1, 0, 2)
            wait_for(lambda c: abs(point(c, 22, 40)[0] - 2) < .005)
            move('reference_wall', 3.3, 0, 2)
            wait_for(lambda c: all(math.isnan(v) for v in point(c, 22, 40)))
            print('PASS: far clipping reports unknown', flush=True)

            with condition:
                received.clear()
            start = time.monotonic()
            wait_for(lambda c: len(received) >= 90, timeout=30)
            with condition:
                samples = list(received)

            def sim(c):
                return c.header.stamp.sec + c.header.stamp.nsec / 1e9
            wall_rate = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
            sim_rate = (len(samples) - 1) / (sim(samples[-1][1]) - sim(samples[0][1]))
            assert 28 <= sim_rate <= 30.1, sim_rate
            print(f'PASS: {sim_rate:.2f} Hz simulation time, {wall_rate:.2f} Hz wall time; '
                  f'{time.monotonic() - start:.1f}s measurement', flush=True)
            print(f'Gazebo log: {output / "gazebo.log"}', flush=True)
        finally:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if process.poll() is not None:
                    break
                os.killpg(process.pid, sig)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    continue
    check_floor(output, simulation, description)


if __name__ == '__main__':
    main()
