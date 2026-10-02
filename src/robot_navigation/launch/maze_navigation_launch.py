#!/usr/bin/env python3
"""Generate a random maze, then launch Gazebo, Nav2, and RViz."""

import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robot_simulation.maze_generator import generate_artifacts


def _generate_and_launch(context):
    """Generate the maze before including the normal navigation launch."""
    simulation_share = get_package_share_directory('robot_simulation')
    navigation_share = get_package_share_directory('robot_navigation')
    seed_text = LaunchConfiguration('seed').perform(context).strip()
    seed = int(seed_text) if seed_text else None
    output_text = LaunchConfiguration('output_dir').perform(context).strip()
    if output_text:
        output_dir = Path(output_text)
    elif seed is not None:
        output_dir = Path('/tmp') / f'flow_maze_seed_{seed}_{time.time_ns()}'
    else:
        output_dir = Path('/tmp') / f'flow_maze_run_{time.time_ns()}'

    artifacts = generate_artifacts(
        template_world=Path(simulation_share) / 'worlds' / 'flow_base.sdf',
        output_dir=output_dir,
        width=int(LaunchConfiguration('width').perform(context)),
        height=int(LaunchConfiguration('height').perform(context)),
        cell_size=float(LaunchConfiguration('cell_size').perform(context)),
        resolution=float(LaunchConfiguration('resolution').perform(context)),
        wall_height=float(LaunchConfiguration('wall_height').perform(context)),
        seed=seed,
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'map': str(artifacts['map']),
            'world': str(artifacts['world']),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )
    info = LogInfo(msg=(
        f"Generated maze seed={artifacts['seed']} "
        f"corridor={artifacts['corridor_width']:.2f} m "
        f"start=({artifacts['start'][0]:.2f}, {artifacts['start'][1]:.2f}) "
        f"goal=({artifacts['goal'][0]:.2f}, {artifacts['goal'][1]:.2f}) "
        f"world={artifacts['world']} "
        f"map={artifacts['map']} "
        f"manifest={artifacts['manifest']}"))
    return [info, navigation_launch]


def generate_launch_description():
    """Return a launch description for a generated maze navigation test."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'seed', default_value='',
            description='Optional integer seed. Empty generates a new maze.',
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description=(
                'Directory for generated artifacts. Empty creates a unique '
                'seed-labeled run directory.'),
        ),
        DeclareLaunchArgument(
            'width', default_value='21',
            description='Odd maze width in cells.',
        ),
        DeclareLaunchArgument(
            'height', default_value='21',
            description='Odd maze height in cells.',
        ),
        DeclareLaunchArgument(
            'cell_size', default_value='2.5',
            description='Corridor width and maze cell size in meters.',
        ),
        DeclareLaunchArgument(
            'resolution', default_value='0.05',
            description='Generated occupancy-map resolution in meters.',
        ),
        DeclareLaunchArgument(
            'wall_height', default_value='0.5',
            description='Gazebo wall height in meters.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the Nav2 display layout.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo time.',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Bring Nav2 lifecycle nodes to active automatically.',
        ),
        OpaqueFunction(function=_generate_and_launch),
    ])
