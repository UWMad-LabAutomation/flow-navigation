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
    'use_imu': ('true', 'Fuse camera gyro with wheel odometry; false uses raw wheel odometry.'),
    'camera_backend': ('cpu', 'Camera simulation: cpu ray casting or rendered depth.'),
    'camera_ray_width': ('80', 'CPU camera horizontal ray samples (not image resolution).'),
    'camera_ray_height': ('45', 'CPU camera vertical ray samples (not image resolution).'),
    'camera_x': ('0.0', 'Camera reference X offset in metres.'),
    'camera_y': ('0.0', 'Camera reference Y offset in metres.'),
    'camera_z': ('2.0', 'Camera reference height in metres; fully raised elevator.'),
    'camera_roll': ('0.0', 'Camera mount roll in radians.'),
    'camera_pitch': ('auto', 'Downward pitch in radians, or auto for ground coverage.'),
    'camera_yaw': ('0.0', 'Camera mount yaw in radians; zero faces +X.'),
    'camera_hfov': ('87.0', 'Depth camera horizontal field of view in degrees.'),
    'camera_vfov': ('58.0', 'Depth camera vertical field of view in degrees.'),
    'camera_image_width': ('1280', 'Depth image width in pixels.'),
    'camera_image_height': ('720', 'Depth image height in pixels.'),
    'camera_fps': ('30.0', 'Requested depth camera frames per simulated second.'),
    'camera_ground_distance': ('1.0', 'Ground distance beyond the front edge for auto pitch.'),
    'camera_near': ('0.3', 'Simulated depth near clipping distance in metres.'),
    'camera_far': ('3.0', 'Simulated depth far clipping distance in metres.'),
    'gui': ('true', 'Open Gazebo GUI; false uses headless rendering.'),
}
CAMERA_POSE_ARGUMENTS = (
    'camera_x', 'camera_y', 'camera_z',
    'camera_roll', 'camera_pitch', 'camera_yaw',
    'camera_hfov', 'camera_vfov', 'camera_ground_distance',
    'camera_image_width', 'camera_image_height',
)
DEPTH_TOPIC = '/model/flow_base/camera/depth/image_raw'
INFO_TOPIC = '/model/flow_base/camera/depth/camera_info'
POINTS_TOPIC = DEPTH_TOPIC + '/points'
IMU_TOPIC = '/model/flow_base/camera/imu'


def sensor_launch_arguments():
    """Declare the same sensor and mount arguments at every launch entry point."""
    return [
        DeclareLaunchArgument(
            name, default_value=default, description=description,
            **({'choices': ['lidar', 'camera']} if name == 'sensor' else
               {'choices': ['cpu', 'render']} if name == 'camera_backend' else {}))
        for name, (default, description) in SENSOR_ARGUMENTS.items()
    ]


def sensor_launch_configurations():
    """Forward the selected sensor settings to an included launch file."""
    return {name: LaunchConfiguration(name) for name in SENSOR_ARGUMENTS}


def _finite_cpu_floor(world):
    """Avoid DART's 2100 m plane box and its inaccurate Bullet ray intersections."""
    collision = world.find("model[@name='ground_plane']/link/collision")
    if collision is None:
        return
    plane = collision.find('geometry/plane')
    if plane is None:
        return
    normal = [float(v) for v in plane.findtext('normal', '0 0 1').split()]
    if normal != [0, 0, 1]:
        raise ValueError('CPU mode requires a +Z ground plane or explicit floor collision mesh')
    size = [float(v) for v in plane.findtext('size', '100 100').split()]
    if len(size) != 2 or not all(math.isfinite(v) and v > 0 for v in size):
        raise ValueError('CPU floor requires two positive finite dimensions')
    pose = collision.find('pose')
    if pose is None:
        pose = ET.SubElement(collision, 'pose')
        pose.text = '0 0 0 0 0 0'
    values = [float(v) for v in pose.text.split()]
    if len(values) != 6 or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('CPU floor collision pose must use six xyz/rpy values')
    roll, pitch, yaw = values[3:]
    if pose.get('degrees', 'false').lower() == 'true':
        roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    # Move the slab center 0.1 m along the negative collision-local normal;
    # its top surface is exactly the original plane, even for a tilted pose.
    direction = (
        math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
        math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
        math.cos(pitch) * math.cos(roll),
    )
    for axis in range(3):
        values[axis] -= 0.1 * direction[axis]
    pose.text = ' '.join(str(v) for v in values)
    geometry = collision.find('geometry')
    geometry.remove(plane)
    ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{size[0]} {size[1]} 0.2'


def configure_sensor_world(world_path, robot_description, sensor,
                           horizontal_fov=87.0, near=0.3, far=3.0,
                           image_width=1280, image_height=720,
                           vertical_fov=58.0, fps=30.0,
                           camera_backend='cpu', ray_width=80, ray_height=45,
                           imu_enabled=False):
    """Create a run-specific world with one sensing source and matching camera TF."""
    if sensor not in ('lidar', 'camera'):
        raise ValueError('sensor must be lidar or camera')
    if camera_backend not in ('cpu', 'render'):
        raise ValueError('camera_backend must be cpu or render')
    if (any(not isinstance(size, int) or size <= 0 for size in (ray_width, ray_height))
            or ray_width * ray_height > 1048576):
        raise ValueError('CPU camera ray dimensions must be positive; at most 1048576 rays')
    if not all(math.isfinite(value)
               for value in (horizontal_fov, vertical_fov, near, far, fps)):
        raise ValueError('camera FOV, clipping distances, and FPS must be finite')
    if not (0.0 < horizontal_fov < 180.0 and 0.0 < vertical_fov < 180.0
            and 0.0 < near < far and fps > 0.0):
        raise ValueError('camera requires FOVs in (0, 180), 0 < near < far, and FPS > 0')
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

    if sensor == 'camera' and camera_backend == 'cpu':
        # The physics ray query uses current collision geometry, including moved
        # and newly inserted objects. No depth camera or GPU lidar is created.
        world = tree.find('world')
        _finite_cpu_floor(world)
        for plugin in list(world.findall('plugin')):
            if plugin.get('name') == 'gz::sim::systems::Sensors':
                world.remove(plugin)
        plugin = ET.SubElement(model, 'plugin', filename='libflow_cpu_camera.so',
                               name='robot_cpu_camera::CpuCamera')
        pose_values = [float(value) for value in pose_values]
        pose_values[2] -= wheel_radius
        for tag, value in {
                'pose': ' '.join(str(value) for value in pose_values),
                'topic': POINTS_TOPIC,
                'width': ray_width, 'height': ray_height,
                'hfov': math.radians(horizontal_fov),
                'vfov': math.radians(vertical_fov),
                'near': near, 'far': far, 'update_rate': fps,
        }.items():
            ET.SubElement(plugin, tag).text = str(value)

    if sensor == 'camera' and camera_backend == 'render':
        camera_sensor = ET.SubElement(
            chassis, 'sensor', name='d435i_depth', type='depth_camera')
        ET.SubElement(camera_sensor, 'pose', relative_to='camera_link').text = (
            '0 0 0 0 0 0')
        ET.SubElement(camera_sensor, 'topic').text = DEPTH_TOPIC
        ET.SubElement(camera_sensor, 'update_rate').text = str(fps)
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
        # Independent focal lengths preserve both specified FOVs at 1280x720.
        # Square-pixel projection from HFOV alone would yield only 56.2 deg VFOV.
        lens = ET.SubElement(camera, 'lens')
        intrinsics = ET.SubElement(lens, 'intrinsics')
        for tag, value in {
                'fx': image_width / (2.0 * math.tan(math.radians(horizontal_fov) / 2.0)),
                'fy': image_height / (2.0 * math.tan(math.radians(vertical_fov) / 2.0)),
                'cx': image_width / 2.0, 'cy': image_height / 2.0, 's': 0.0,
        }.items():
            ET.SubElement(intrinsics, tag).text = str(value)
        clip = ET.SubElement(camera, 'clip')
        ET.SubElement(clip, 'near').text = str(near)
        ET.SubElement(clip, 'far').text = str(far)

    if imu_enabled:
        # The dedicated Imu system is physics based and does not render images.
        world = tree.find('world')
        ET.SubElement(world, 'plugin', filename='gz-sim-imu-system',
                      name='gz::sim::systems::Imu')
        imu = ET.SubElement(chassis, 'sensor', name='d435i_imu', type='imu')
        ET.SubElement(imu, 'pose', relative_to='camera_link').text = '0 0 0 0 0 0'
        ET.SubElement(imu, 'topic').text = IMU_TOPIC
        ET.SubElement(imu, 'update_rate').text = '100'
        ET.SubElement(imu, 'always_on').text = 'true'
        settings = ET.SubElement(imu, 'imu')
        for quantity, deviation in [('angular_velocity', 0.001), ('linear_acceleration', 0.01)]:
            axes = ET.SubElement(settings, quantity)
            for axis in ('x', 'y', 'z'):
                noise = ET.SubElement(ET.SubElement(axes, axis), 'noise', type='gaussian')
                ET.SubElement(noise, 'mean').text = '0'
                ET.SubElement(noise, 'stddev').text = str(deviation)

    output_dir = Path(tempfile.mkdtemp(prefix='flow_sensors_'))
    output_path = output_dir / world_path.name
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    return output_path
