#!/usr/bin/env python3
"""Start the FLOW simulation, holonomic Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Return a complete map-based navigation launch description."""
    navigation_share = get_package_share_directory('robot_navigation')
    simulation_share = get_package_share_directory('robot_simulation')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world')
    sim_time_parameter = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }
    common_parameters = [params_file, sim_time_parameter]

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_localization = ['map_server', 'amcl']
    lifecycle_navigation = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share, 'launch', 'flow_base_sim.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            params_file,
            {
                'yaml_filename': map_file,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        remappings=tf_remappings,
        output='screen',
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        output='screen',
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=common_parameters,
        remappings=tf_remappings,
        output='screen',
    )

    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[
            {
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': lifecycle_localization,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        output='screen',
    )

    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[
            {
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': lifecycle_navigation,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        output='screen',
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                nav2_share, 'maps', 'tb3_sandbox.yaml'),
            description='Absolute path to the imported map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_share, 'config', 'nav2_flow_params.yaml'),
            description='Nav2 parameters for the holonomic FLOW base.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                simulation_share, 'worlds', 'flow_base.sdf'),
            description='SDF world file to load in Gazebo.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo time.',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Bring Nav2 lifecycle nodes to active automatically.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the Nav2 display layout.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                nav2_share, 'rviz', 'nav2_default_view.rviz'),
            description='RViz configuration file.',
        ),
        simulation,
        map_server,
        amcl,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        localization_manager,
        navigation_manager,
        rviz,
    ])
