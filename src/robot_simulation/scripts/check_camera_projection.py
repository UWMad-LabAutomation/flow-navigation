#!/usr/bin/env python3
# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check rendered D435i depth and cloud geometry against a wall 2 m away."""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.transport13 import Node
from robot_simulation.sensor_configuration import (
    configure_sensor_world, DEPTH_TOPIC, INFO_TOPIC, POINTS_TOPIC,
)
import xacro


def main():
    """Verify all image edges, not just the center ray or CameraInfo metadata."""
    output = Path(tempfile.mkdtemp(prefix='flow_camera_projection_'))
    simulation = Path(get_package_share_directory('robot_simulation'))
    description = Path(get_package_share_directory('robot_description'))
    renderer = Path(get_package_prefix('robot_simulation')) / 'opt/camera_renderer'
    if not (renderer / 'flow_projection_fix.txt').is_file():
        raise RuntimeError('Run scripts/build_camera_renderer.py first')
    world = configure_sensor_world(
        simulation / 'worlds/flow_base.sdf',
        xacro.process_file(str(description / 'urdf/flow_base.urdf.xacro'),
                           mappings={'camera_pitch': '0'}).toxml(), 'camera',
        camera_backend='render')
    tree = ET.parse(world)
    model = ET.SubElement(tree.find('world'), 'model', name='projection_test_wall')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = '2.1 0 2 0 0 0'
    link = ET.SubElement(model, 'link', name='wall')
    for kind in ('visual', 'collision'):
        geometry = ET.SubElement(ET.SubElement(link, kind, name='wall'), 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = '0.2 5 5'
    tree.write(world)

    os.environ['GZ_PARTITION'] = f'flow_projection_check_{os.getpid()}'
    os.environ['GZ_RENDERING_PLUGIN_PATH'] = str(renderer / 'lib/gz-rendering-8/engine-plugins')
    os.environ['LD_LIBRARY_PATH'] = os.pathsep.join([
        os.environ['GZ_RENDERING_PLUGIN_PATH'], str(renderer / 'lib'),
        os.environ.get('LD_LIBRARY_PATH', ''),
    ])
    node = Node()
    ready = threading.Event()
    received = {}

    def receive(key, message):
        received.setdefault(key, message)
        if len(received) == 3:
            ready.set()

    node.subscribe(CameraInfo, INFO_TOPIC, lambda message: receive('info', message))
    node.subscribe(Image, DEPTH_TOPIC, lambda message: receive('image', message))
    node.subscribe(PointCloudPacked, POINTS_TOPIC, lambda message: receive('cloud', message))
    with (output / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', '--headless-rendering', str(world)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if not ready.wait(timeout=50):
                raise RuntimeError(f'Missing camera messages; inspect {output / "gazebo.log"}')
            info, image, cloud = [received[key] for key in ('info', 'image', 'cloud')]
            assert (info.width, info.height) == (1280, 720)
            assert (image.width, image.height) == (1280, 720)
            assert (cloud.width, cloud.height) == (1280, 720)
            k = info.intrinsics.k
            hfov = math.degrees(2 * math.atan(info.width / (2 * k[0])))
            vfov = math.degrees(2 * math.atan(info.height / (2 * k[4])))
            assert abs(hfov - 87) < 1e-6 and abs(vfov - 58) < 1e-6
            # The rectified projection must agree with the configured lens.
            assert abs(info.projection.p[0] - k[0]) < 1e-3, (info.projection.p[0], k[0])
            assert abs(info.projection.p[5] - k[4]) < 1e-3, (info.projection.p[5], k[4])
            assert not cloud.is_bigendian
            fields = {field.name: field.offset for field in cloud.field}
            for row, col in [(360, 640), (360, 0), (360, 1279), (0, 640), (719, 640)]:
                base = row * cloud.row_step + col * cloud.point_step
                xyz = [struct.unpack_from('<f', cloud.data, base + fields[axis])[0]
                       for axis in ('x', 'y', 'z')]
                # Gazebo cloud uses X forward, Y left, Z up; rays pass through
                # pixel centers. The entire visible wall face is at X=2 m.
                expected = [2.0, 2 * (k[2] - col - 0.5) / k[0],
                            2 * (k[5] - row - 0.5) / k[4]]
                depth = struct.unpack_from('<f', image.data, row * image.step + col * 4)[0]
                assert abs(depth - 2) < 0.005, (row, col, depth)
                assert all(abs(a - b) < 0.005 for a, b in zip(xyz, expected)), (
                    row, col, xyz, expected)
            print(f'PASS: {info.width}x{info.height}, {hfov:.1f}x{vfov:.1f} deg; '
                  'center and all four cloud edges agree with a wall at 2 m.')
            print(f'Gazebo log: {output / "gazebo.log"}')
        finally:
            # Stop only the simulation process group created by this check.
            for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if process.poll() is not None:
                    break
                os.killpg(process.pid, stop_signal)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    continue


if __name__ == '__main__':
    main()
