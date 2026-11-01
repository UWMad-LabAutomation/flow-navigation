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
        artifacts['world'], description(), 'camera', camera_backend='render')
    tree = ET.parse(configured)
    before = ET.fromstring(original)
    model = tree.find("world/model[@name='flow_base']")
    sensors = model.findall('.//sensor')
    assert [sensor.get('type') for sensor in sensors] == ['depth_camera']
    assert sensors[0].findtext('camera/optical_frame_id') == 'camera_depth_optical_frame'
    assert sensors[0].findtext('camera/image/width') == '1280'
    assert sensors[0].findtext('camera/image/height') == '720'
    assert float(sensors[0].findtext('update_rate')) == 30.0
    assert float(sensors[0].findtext('camera/clip/near')) == 0.3
    assert float(sensors[0].findtext('camera/clip/far')) == 3.0
    intrinsics = sensors[0].find('camera/lens/intrinsics')
    fx = float(intrinsics.findtext('fx'))
    fy = float(intrinsics.findtext('fy'))
    assert math.degrees(2 * math.atan(640 / fx)) == pytest.approx(87)
    assert math.degrees(2 * math.atan(360 / fy)) == pytest.approx(58)
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
        WORLD, robot_xml, 'camera', horizontal_fov=90, vertical_fov=60,
        image_width=640, image_height=360, fps=15, camera_backend='render')
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
    fy = float(camera_sensor.findtext('camera/lens/intrinsics/fy'))
    assert math.degrees(2 * math.atan(180 / fy)) == pytest.approx(60)
    assert float(camera_sensor.findtext('update_rate')) == 15


@pytest.mark.parametrize('height,hfov,vfov,width,pixels_high', [
    (2.0, 87, 58, 1280, 720), (1.5, 87, 58, 640, 360), (2.0, 90, 60, 320, 240),
])
def test_auto_pitch_sees_ground_one_metre_past_front(height, hfov, vfov, width, pixels_high):
    """The lowest center-column ray must reach the requested ground point."""
    robot = ET.fromstring(description(
        camera_z=str(height), camera_hfov=str(hfov),
        camera_vfov=str(vfov),
        camera_image_width=str(width), camera_image_height=str(pixels_high)))
    mount = robot.find("joint[@name='base_link_to_camera_link']/origin")
    pitch = float(mount.get('rpy').split()[1])
    half_vfov = math.radians(vfov) / 2
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


def test_camera_imu_uses_mount_without_rendering():
    """The physics IMU and ROS frame must use the same adjustable camera mount."""
    robot_xml = description(camera_z='1.5', camera_pitch='0.3', camera_roll='0.1')
    configured = configure_sensor_world(WORLD, robot_xml, 'camera', imu_enabled=True)
    tree = ET.parse(configured)
    imu = tree.find("world/model/link/sensor[@type='imu']")
    assert imu is not None
    assert imu.find('pose').get('relative_to') == 'camera_link'
    assert imu.findtext('update_rate') == '100'
    assert imu.findtext('imu/angular_velocity/z/noise/stddev') == '0.001'
    assert tree.find("world/plugin[@name='gz::sim::systems::Imu']") is not None
    assert tree.find("world/plugin[@name='gz::sim::systems::Sensors']") is None
    assert tree.findall(".//sensor[@type='gpu_lidar']") == []
    robot = ET.fromstring(robot_xml)
    joint = robot.find("joint[@name='camera_link_to_imu_frame']")
    assert joint.find('parent').get('link') == 'camera_link'
    assert joint.find('origin').get('rpy') == '0 0 0'


def test_driver_can_own_nominal_camera_frames():
    """Hardware driver extrinsics must not compete with simulated camera frames."""
    robot = ET.fromstring(description(camera_nominal_depth_frames='false'))
    assert robot.find("link[@name='camera_imu_frame']") is None
    assert robot.find("link[@name='camera_depth_frame']") is None


def test_cpu_camera_preserves_world_and_removes_rendered_sensors(tmp_path):
    """CPU mode samples the same world without a renderer or a lidar sensor."""
    artifacts = generate_artifacts(WORLD, tmp_path, width=5, height=5, seed=19)
    before = ET.parse(artifacts['world'])
    configured = configure_sensor_world(
        artifacts['world'], description(camera_z='1.5', camera_pitch='0.3'), 'camera',
        ray_width=80, ray_height=45, fps=20)
    tree = ET.parse(configured)
    assert tree.findall('.//sensor') == []
    assert tree.find("world/plugin[@name='gz::sim::systems::Sensors']") is None
    floor = tree.find("world/model[@name='ground_plane']/link/collision")
    assert floor.find('geometry/plane') is None
    assert [float(v) for v in floor.findtext('geometry/box/size').split()] == [100, 100, 0.2]
    assert [float(v) for v in floor.findtext('pose').split()] == [0, 0, -0.1, 0, 0, 0]
    original_floor = before.find("world/model[@name='ground_plane']/link/collision/geometry/plane")
    assert original_floor is not None
    plugin = tree.find("world/model/plugin[@name='robot_cpu_camera::CpuCamera']")
    assert plugin is not None
    assert plugin.findtext('width') == '80'
    assert plugin.findtext('height') == '45'
    assert plugin.findtext('update_rate') == '20'
    assert [float(v) for v in plugin.findtext('pose').split()] == [0, 0, 1.42, 0, 0.3, 0]
    assert math.degrees(float(plugin.findtext('hfov'))) == pytest.approx(87)
    assert math.degrees(float(plugin.findtext('vfov'))) == pytest.approx(58)
    assert float(plugin.findtext('near')) == 0.3
    assert float(plugin.findtext('far')) == 3.0
    for xpath in ('world/physics', "world/model[@name='maze_walls']",
                  "world/model[@name='flow_base']/pose",
                  "world/model/plugin[@name='gz::sim::systems::MecanumDrive']"):
        assert ET.tostring(tree.find(xpath)) == ET.tostring(before.find(xpath))


@pytest.mark.parametrize('options', [
    {'sensor': 'unknown'}, {'sensor': 'camera', 'horizontal_fov': 180},
    {'sensor': 'camera', 'near': 4, 'far': 3},
    {'sensor': 'camera', 'horizontal_fov': float('nan')},
    {'sensor': 'camera', 'vertical_fov': 0},
    {'sensor': 'camera', 'vertical_fov': float('nan')},
    {'sensor': 'camera', 'fps': 0},
    {'sensor': 'camera', 'fps': float('inf')},
    {'sensor': 'camera', 'image_width': 0},
    {'sensor': 'camera', 'image_height': 180.5},
    {'sensor': 'camera', 'camera_backend': 'invalid'},
    {'sensor': 'camera', 'ray_width': 0},
    {'sensor': 'camera', 'ray_height': 1.5},
    {'sensor': 'camera', 'ray_width': 2000, 'ray_height': 2000},
])
def test_invalid_camera_options_are_rejected(options):
    """Reject invalid choices before starting Gazebo."""
    with pytest.raises(ValueError):
        configure_sensor_world(WORLD, description(), **options)
