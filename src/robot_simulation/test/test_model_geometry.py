# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Check agreement between the robot model, physics geometry, and navigation."""

import ast
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from robot_simulation.sensor_configuration import configure_sensor_world
import xacro
import yaml


SOURCE = Path(__file__).resolve().parents[2]
WORLD = SOURCE / 'robot_simulation/worlds/flow_base.sdf'
DESCRIPTION = SOURCE / 'robot_description/urdf/flow_base.urdf.xacro'
PARAMETERS = SOURCE / 'robot_navigation/config/nav2_flow_params.yaml'


def numbers(text):
    """Read an XML vector."""
    return [float(value) for value in text.split()]


def rotate(point, rpy):
    """Apply the URDF fixed-axis roll, pitch, yaw convention to a point."""
    x, y, z = point
    roll, pitch, yaw = rpy
    y, z = math.cos(roll) * y - math.sin(roll) * z, (
        math.sin(roll) * y + math.cos(roll) * z)
    x, z = math.cos(pitch) * x + math.sin(pitch) * z, (
        -math.sin(pitch) * x + math.cos(pitch) * z)
    return (math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y, z)


@pytest.fixture(scope='module')
def robot():
    """Use the actual default robot description."""
    return ET.fromstring(xacro.process_file(str(DESCRIPTION)).toxml())


@pytest.fixture(scope='module')
def navigation_parameters():
    """Load both real costmap configurations."""
    return yaml.safe_load(PARAMETERS.read_text())


def footprint_bounds(parameters, costmap):
    """Return bounds after confirming the configured footprint is a rectangle."""
    settings = parameters[costmap][costmap]['ros__parameters']
    footprint = ast.literal_eval(settings['footprint'])
    xs, ys = zip(*footprint)
    assert set(map(tuple, footprint)) == set(itertools.product(
        (min(xs), max(xs)), (min(ys), max(ys))))
    return min(xs), max(xs), min(ys), max(ys)


def box_pose(robot, name):
    """Get a directly mounted box center and dimensions in the ground frame."""
    body = robot.find(f"link[@name='{name}']/collision")
    origin = body.find('origin')
    offset = numbers(origin.get('xyz'))
    assert numbers(origin.get('rpy')) == [0, 0, 0]
    if name != 'base_link':
        joint = robot.find(f"joint[@name='base_link_to_{name}']")
        assert joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') == 'base_link'
        mount = joint.find('origin')
        assert numbers(mount.get('rpy')) == [0, 0, 0]
        offset = [a + b for a, b in zip(offset, numbers(mount.get('xyz')))]
    return offset, numbers(body.find('geometry/box').get('size'))


@pytest.mark.parametrize('backend', ['cpu', 'render'])
def test_urdf_and_gazebo_boxes_have_same_ground_geometry(robot, backend):
    """A wheel-center model origin must not raise the rails above their ROS TF."""
    world = ET.parse(configure_sensor_world(
        WORLD, ET.tostring(robot, encoding='unicode'), 'camera', camera_backend=backend))
    model = world.find("world/model[@name='flow_base']")
    chassis = model.find("link[@name='base_link']")
    model_z = numbers(model.findtext('pose'))[2]
    chassis_z = numbers(chassis.findtext('pose'))[2]
    center, size = box_pose(robot, 'base_link')
    assert size == pytest.approx([0.54, 0.54, 0.20])
    assert center[2] == pytest.approx(model_z + chassis_z)
    for kind in ('visual', 'collision'):
        assert numbers(chassis.findtext(f'{kind}/geometry/box/size')) == pytest.approx(size)

    navigation = model.find("frame[@name='navigation_base']/pose")
    assert navigation.get('relative_to') == '__model__'
    navigation_z = model_z + numbers(navigation.text)[2]
    assert navigation_z == pytest.approx(0)
    for name in ('rail_support_link', 'arm_rail_link'):
        center, size = box_pose(robot, name)
        frame = model.find(f"frame[@name='{name}']/pose")
        assert frame.get('relative_to') == 'navigation_base'
        frame_pose = numbers(frame.text)
        assert frame_pose[:3] == pytest.approx(center)
        assert frame_pose[3:] == [0, 0, 0]
        for kind in ('visual', 'collision'):
            body = chassis.find(f"{kind}[@name='{name}']")
            pose = body.find('pose')
            assert pose.get('relative_to') == name
            assert numbers(pose.text) == [0, 0, 0, 0, 0, 0]
            assert numbers(body.findtext('geometry/box/size')) == pytest.approx(size)


def test_upright_connects_base_to_rail(robot):
    """The transverse rail setback and full-depth support must fit the base."""
    base_center, base_size = box_pose(robot, 'base_link')
    support_center, support_size = box_pose(robot, 'rail_support_link')
    rail_center, rail_size = box_pose(robot, 'arm_rail_link')
    assert rail_center == pytest.approx([0.20, 0, 1.25])
    assert rail_size == pytest.approx([0.10, 0.75, 0.05])
    assert support_center == pytest.approx([0.15, 0, 0.7525])
    assert support_size == pytest.approx([0.20, 0.15, 0.945])
    base_front = base_center[0] + base_size[0] / 2
    rail_front = rail_center[0] + rail_size[0] / 2
    assert base_front - rail_front == pytest.approx(0.02)
    assert support_center[0] + support_size[0] / 2 == pytest.approx(rail_front)
    assert support_center[0] - support_size[0] / 2 <= rail_center[0] - rail_size[0] / 2
    for axis in (0, 1):
        assert support_center[axis] - support_size[axis] / 2 >= (
            base_center[axis] - base_size[axis] / 2)
        assert support_center[axis] + support_size[axis] / 2 <= (
            base_center[axis] + base_size[axis] / 2)
    assert support_center[2] - support_size[2] / 2 == pytest.approx(
        base_center[2] + base_size[2] / 2)
    assert support_center[2] + support_size[2] / 2 == pytest.approx(
        rail_center[2] - rail_size[2] / 2)


def test_wheel_collisions_fit_footprint_and_match_drive(navigation_parameters):
    """Drive kinematics and clearance must account for actual collision spheres."""
    model = ET.parse(WORLD).find("world/model[@name='flow_base']")
    drive = model.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    bounds = footprint_bounds(navigation_parameters, 'local_costmap')
    wheel_centers = {}
    for name in ('front_left_wheel', 'front_right_wheel',
                 'rear_left_wheel', 'rear_right_wheel'):
        link = model.find(f"link[@name='{name}']")
        center = numbers(link.findtext('pose'))[:3]
        radius = float(link.findtext('collision/geometry/sphere/radius'))
        assert radius == pytest.approx(float(drive.findtext('wheel_radius')))
        assert center[0] - radius >= bounds[0] - 1e-12
        assert center[0] + radius <= bounds[1] + 1e-12
        assert center[1] - radius >= bounds[2] - 1e-12
        assert center[1] + radius <= bounds[3] + 1e-12
        assert abs(center[0]) + radius <= 0.27 + 1e-12
        assert abs(center[1]) + radius <= 0.27 + 1e-12
        wheel_centers[name] = center
    front_left = wheel_centers['front_left_wheel']
    front_right = wheel_centers['front_right_wheel']
    rear_left = wheel_centers['rear_left_wheel']
    assert float(drive.findtext('wheelbase')) == pytest.approx(
        front_left[0] - rear_left[0])
    assert float(drive.findtext('wheel_separation')) == pytest.approx(
        front_left[1] - front_right[1])


def test_costmaps_contain_base_rail_support_and_pitched_camera(robot, navigation_parameters):
    """Both costmaps must protect upper-body overhang without relying on padding."""
    local = footprint_bounds(navigation_parameters, 'local_costmap')
    assert local == pytest.approx((-0.27, 0.30, -0.375, 0.375))
    assert footprint_bounds(navigation_parameters, 'global_costmap') == local
    for name in ('base_link', 'rail_support_link', 'arm_rail_link', 'camera_link'):
        body = robot.find(f"link[@name='{name}']/collision")
        size = numbers(body.find('geometry/box').get('size'))
        geometry_origin = body.find('origin')
        geometry_xyz = numbers(geometry_origin.get('xyz'))
        geometry_rpy = numbers(geometry_origin.get('rpy'))
        mount_xyz, mount_rpy = [0, 0, 0], [0, 0, 0]
        if name != 'base_link':
            mount = robot.find(f"joint[@name='base_link_to_{name}']/origin")
            mount_xyz, mount_rpy = numbers(mount.get('xyz')), numbers(mount.get('rpy'))
        for signs in itertools.product((-1, 1), repeat=3):
            corner = rotate([s * length / 2 for s, length in zip(signs, size)], geometry_rpy)
            corner = rotate([a + b for a, b in zip(corner, geometry_xyz)], mount_rpy)
            x, y, _ = [a + b for a, b in zip(corner, mount_xyz)]
            assert local[0] - 1e-12 <= x <= local[1] + 1e-12, (name, x)
            assert local[2] - 1e-12 <= y <= local[3] + 1e-12, (name, y)


def test_default_camera_frustum_clears_structure_and_uses_tf_pose(robot):
    """Every forward ray must clear the rail and share the ROS camera pose."""
    world = ET.parse(configure_sensor_world(
        WORLD, ET.tostring(robot, encoding='unicode'), 'camera', imu_enabled=True))
    model = world.find("world/model[@name='flow_base']")
    plugin = model.find("plugin[@name='robot_cpu_camera::CpuCamera']")
    mount = robot.find("joint[@name='base_link_to_camera_link']/origin")
    position, orientation = numbers(mount.get('xyz')), numbers(mount.get('rpy'))
    assert position == pytest.approx([0.275, 0, 1.25])
    assert orientation[0] == 0 and orientation[2] == 0
    camera_frame = model.find("frame[@name='camera_link']/pose")
    assert camera_frame.get('relative_to') == 'navigation_base'
    assert numbers(camera_frame.text) == pytest.approx(position + orientation)
    cpu_pose = numbers(plugin.findtext('pose'))
    cpu_pose[2] += numbers(model.findtext('pose'))[2]
    assert cpu_pose == pytest.approx(position + orientation)
    imu_pose = model.find("link/sensor[@type='imu']/pose")
    assert imu_pose.get('relative_to') == 'camera_link'
    assert numbers(imu_pose.text) == [0, 0, 0, 0, 0, 0]

    for name in ('base_link', 'rail_support_link', 'arm_rail_link'):
        center, size = box_pose(robot, name)
        assert position[0] > center[0] + size[0] / 2
    half_hfov = float(plugin.findtext('hfov')) / 2
    half_vfov = float(plugin.findtext('vfov')) / 2
    for horizontal, vertical in itertools.product((-1, 1), repeat=2):
        ray = rotate((1, horizontal * math.tan(half_hfov),
                      vertical * math.tan(half_vfov)), orientation)
        assert ray[0] > 0
    lower_ray = rotate((1, 0, -math.tan(half_vfov)), orientation)
    ground_x = position[0] - position[2] * lower_ray[0] / lower_ray[2]
    assert ground_x == pytest.approx(0.54 / 2 + 1.0)
