# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check sensor selection, camera geometry, and preservation of generated mazes."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from robot_simulation.maze_generator import generate_artifacts
from robot_simulation.sensor_configuration import configure_sensor_world
import xacro


SOURCE = Path(__file__).resolve().parents[2]
WORLD = SOURCE / 'robot_simulation/worlds/flow_base.sdf'
DESCRIPTION = SOURCE / 'robot_description/urdf/flow_base.urdf.xacro'


def description(**pose):
    """Expand the real description with the requested mount pose."""
    return xacro.process_file(str(DESCRIPTION), mappings=pose).toxml()


def test_camera_replaces_lidar_and_preserves_maze(tmp_path):
    """Selecting the camera must retain the same walls, robot spawn, and physics."""
    artifacts = generate_artifacts(WORLD, tmp_path, width=5, height=5, seed=19)
    original = artifacts['world'].read_bytes()
    configured = configure_sensor_world(
        artifacts['world'], description(), 'camera')
    tree = ET.parse(configured)
    before = ET.fromstring(original)
    model = tree.find("world/model[@name='flow_base']")
    sensors = model.findall('.//sensor')
    assert [sensor.get('type') for sensor in sensors] == ['depth_camera']
    assert sensors[0].findtext('camera/optical_frame_id') == 'camera_depth_optical_frame'
    assert sensors[0].findtext('camera/image/width') == '320'
    assert sensors[0].findtext('camera/image/height') == '180'
    size = model.findtext("link/visual[@name='d435i_body']/geometry/box/size")
    assert [float(value) for value in size.split()] == [0.025, 0.09, 0.025]
    for xpath in ('world/physics', "world/model[@name='maze_walls']",
                  "world/model[@name='flow_base']/pose",
                  "world/model[@name='flow_base']/plugin"):
        assert ET.tostring(tree.find(xpath)) == ET.tostring(before.find(xpath))
    assert artifacts['world'].read_bytes() == original


def test_camera_pose_and_fov_follow_arguments():
    """Camera rendering must use the same pose as TF, including height and pitch."""
    robot_xml = description(camera_x='0.15', camera_z='1.4', camera_pitch='0.3')
    configured = configure_sensor_world(
        WORLD, robot_xml, 'camera', horizontal_fov=90, image_width=640, image_height=360)
    tree = ET.parse(configured)
    frame = tree.find("world/model/frame[@name='camera_link']/pose")
    assert frame.get('relative_to') == 'navigation_base'
    ground_frame = tree.find("world/model/frame[@name='navigation_base']/pose")
    assert ground_frame.get('relative_to') == '__model__'
    assert [float(value) for value in ground_frame.text.split()] == [0, 0, -0.08, 0, 0, 0]
    assert [float(value) for value in frame.text.split()] == [0.15, 0, 1.4, 0, 0.3, 0]
    camera_sensor = tree.find("world/model/link/sensor[@type='depth_camera']")
    assert camera_sensor.find('pose').get('relative_to') == 'camera_link'
    assert float(camera_sensor.findtext('camera/horizontal_fov')) == pytest.approx(math.pi / 2)
    assert camera_sensor.findtext('camera/image/width') == '640'
    assert camera_sensor.findtext('camera/image/height') == '360'


@pytest.mark.parametrize('height,hfov,width,pixels_high', [
    (1.7, 90, 320, 180), (1.5, 87, 640, 360), (2.0, 90, 320, 240),
])
def test_auto_pitch_sees_ground_one_metre_past_front(height, hfov, width, pixels_high):
    """The lowest center-column ray must reach the requested ground point."""
    robot = ET.fromstring(description(
        camera_z=str(height), camera_hfov=str(hfov),
        camera_image_width=str(width), camera_image_height=str(pixels_high)))
    mount = robot.find("joint[@name='base_link_to_camera_link']/origin")
    pitch = float(mount.get('rpy').split()[1])
    half_vfov = math.atan(math.tan(math.radians(hfov) / 2) * pixels_high / width)
    ground_x = height / math.tan(pitch + half_vfov)
    assert ground_x - 0.751 / 2 == pytest.approx(1.0)
    scan = robot.find("joint[@name='base_link_to_camera_scan_frame']/origin")
    assert [float(value) for value in scan.get('rpy').split()] == [0, 0, 0]
    assert float(scan.get('xyz').split()[2]) == pytest.approx(0.22)


def test_lidar_mode_preserves_existing_scan_sensor():
    """The existing navigation sensing mode must keep its original scan settings."""
    configured = configure_sensor_world(WORLD, description(), 'lidar')
    before = ET.parse(WORLD).find("world/model/link/sensor[@type='gpu_lidar']")
    after = ET.parse(configured).findall('world/model/link/sensor')
    assert len(after) == 1
    assert ET.tostring(after[0]) == ET.tostring(before)


@pytest.mark.parametrize('options', [
    {'sensor': 'unknown'}, {'sensor': 'camera', 'horizontal_fov': 180},
    {'sensor': 'camera', 'near': 4, 'far': 3},
    {'sensor': 'camera', 'horizontal_fov': float('nan')},
    {'sensor': 'camera', 'image_width': 0},
    {'sensor': 'camera', 'image_height': 180.5},
])
def test_invalid_camera_options_are_rejected(options):
    """Reject invalid choices before starting Gazebo."""
    with pytest.raises(ValueError):
        configure_sensor_world(WORLD, description(), **options)
