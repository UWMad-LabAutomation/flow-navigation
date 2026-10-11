# Copyright 2026 Zach Gaier
# Licensed under the Apache License, Version 2.0.

"""Configure the FLOW world's selected sensor without changing its source file."""

import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


SENSOR_ARGUMENTS = {
    'sensor': ('camera', 'Gazebo sensing source: camera or legacy lidar.'),
    'camera_x': ('0.0', 'Camera reference X offset in metres.'),
    'camera_y': ('0.0', 'Camera reference Y offset in metres.'),
    'camera_z': ('1.70', 'Camera reference height in metres; provisional.'),
    'camera_roll': ('0.0', 'Camera mount roll in radians.'),
    'camera_pitch': ('auto', 'Downward pitch in radians, or auto for ground coverage.'),
    'camera_yaw': ('0.0', 'Camera mount yaw in radians; zero faces +X.'),
    'camera_hfov': ('90.0', 'Depth camera horizontal field of view in degrees.'),
    'camera_image_width': ('320', 'Depth image width in pixels; controls point-cloud load.'),
    'camera_image_height': ('180', 'Depth image height in pixels; also determines vertical FOV.'),
    'camera_ground_distance': ('1.0', 'Ground distance beyond the front edge for auto pitch.'),
    'camera_near': ('0.28', 'Simulated depth near clipping distance in metres.'),
    'camera_far': ('3.0', 'Simulated depth far clipping distance in metres.'),
    'gui': ('true', 'Open Gazebo GUI; false uses headless rendering.'),
}
CAMERA_POSE_ARGUMENTS = (
    'camera_x', 'camera_y', 'camera_z',
    'camera_roll', 'camera_pitch', 'camera_yaw',
    'camera_hfov', 'camera_ground_distance',
    'camera_image_width', 'camera_image_height',
)
DEPTH_TOPIC = '/model/flow_base/camera/depth/image_raw'
INFO_TOPIC = '/model/flow_base/camera/depth/camera_info'
POINTS_TOPIC = DEPTH_TOPIC + '/points'


def sensor_launch_arguments():
    """Declare the same sensor and mount arguments at every launch entry point."""
    return [
        DeclareLaunchArgument(
            name, default_value=default, description=description,
            **({'choices': ['lidar', 'camera']} if name == 'sensor' else {}))
        for name, (default, description) in SENSOR_ARGUMENTS.items()
    ]


def sensor_launch_configurations():
    """Forward the selected sensor settings to an included launch file."""
    return {name: LaunchConfiguration(name) for name in SENSOR_ARGUMENTS}


def configure_sensor_world(world_path, robot_description, sensor,
                           horizontal_fov=90.0, near=0.28, far=3.0,
                           image_width=320, image_height=180):
    """Create a run-specific world with one sensing source and matching camera TF."""
    if sensor not in ('lidar', 'camera'):
        raise ValueError('sensor must be lidar or camera')
    if not all(math.isfinite(value) for value in (horizontal_fov, near, far)):
        raise ValueError('camera FOV and clipping distances must be finite')
    if not 0.0 < horizontal_fov < 180.0 or not 0.0 < near < far:
        raise ValueError('camera requires 0 < hfov < 180 and 0 < near < far')
    if any(not isinstance(size, int) or size <= 0
           for size in (image_width, image_height)):
        raise ValueError('camera image dimensions must be positive integers')

    # Preserve Gazebo's namespaced friction attributes and world/maze geometry.
    ET.register_namespace('gz', 'http://gazebosim.org/schema')
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    world_path = Path(world_path).resolve()
    tree = ET.parse(world_path, parser=parser)
    model = tree.find("./world/model[@name='flow_base']")
    if model is None:
        raise ValueError('world must contain a model named flow_base')
    chassis = model.find("link[@name='base_link']")
    lidar_link = model.find("link[@name='laser_link']")
    if chassis is None or lidar_link is None:
        raise ValueError('FLOW world must contain base_link and laser_link')

    robot = ET.fromstring(robot_description)
    mount = robot.find("joint[@name='base_link_to_camera_link']/origin")
    body = robot.find("link[@name='camera_link']/visual")
    if mount is None or body is None:
        raise ValueError('robot description must contain the D435i camera mount')
    pose_values = (mount.get('xyz') + ' ' + mount.get('rpy')).split()
    if len(pose_values) != 6 or not all(
            math.isfinite(float(value)) for value in pose_values):
        raise ValueError('camera mount must contain six finite pose values')

    # ROS base_footprint/base_link is on the ground (planar odometry has Z=0).
    # The FLOW SDF model origin is at the wheel centers, one radius above it.
    wheel_radius = float(model.findtext(
        "plugin[@name='gz::sim::systems::MecanumDrive']/wheel_radius", 'nan'))
    if not math.isfinite(wheel_radius) or wheel_radius <= 0:
        raise ValueError('FLOW world must define a positive MecanumDrive wheel_radius')
    navigation_frame = ET.SubElement(
        model, 'frame', name='navigation_base', attached_to='base_link')
    ET.SubElement(navigation_frame, 'pose', relative_to='__model__').text = (
        f'0 0 {-wheel_radius} 0 0 0')
    camera_frame = ET.SubElement(
        model, 'frame', name='camera_link', attached_to='base_link')
    ET.SubElement(camera_frame, 'pose', relative_to='navigation_base').text = (
        mount.get('xyz') + ' ' + mount.get('rpy'))
    visual = ET.SubElement(chassis, 'visual', name='d435i_body')
    origin = body.find('origin')
    ET.SubElement(visual, 'pose', relative_to='camera_link').text = (
        origin.get('xyz') + ' ' + origin.get('rpy'))
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    ET.SubElement(box, 'size').text = body.find('geometry/box').get('size')
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = '0.65 0.65 0.68 1'

    if sensor == 'camera':
        for lidar in list(lidar_link.findall('sensor')):
            lidar_link.remove(lidar)
        camera_sensor = ET.SubElement(
            chassis, 'sensor', name='d435i_depth', type='depth_camera')
        ET.SubElement(camera_sensor, 'pose', relative_to='camera_link').text = (
            '0 0 0 0 0 0')
        ET.SubElement(camera_sensor, 'topic').text = DEPTH_TOPIC
        ET.SubElement(camera_sensor, 'update_rate').text = '10'
        ET.SubElement(camera_sensor, 'always_on').text = 'true'
        camera = ET.SubElement(camera_sensor, 'camera')
        ET.SubElement(camera, 'horizontal_fov').text = str(
            math.radians(horizontal_fov))
        ET.SubElement(camera, 'optical_frame_id').text = (
            'camera_depth_optical_frame')
        ET.SubElement(camera, 'camera_info_topic').text = INFO_TOPIC
        image = ET.SubElement(camera, 'image')
        ET.SubElement(image, 'width').text = str(image_width)
        ET.SubElement(image, 'height').text = str(image_height)
        ET.SubElement(image, 'format').text = 'R_FLOAT32'
        clip = ET.SubElement(camera, 'clip')
        ET.SubElement(clip, 'near').text = str(near)
        ET.SubElement(clip, 'far').text = str(far)

    output_dir = Path(tempfile.mkdtemp(prefix='flow_sensors_'))
    output_path = output_dir / world_path.name
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    return output_path
