#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_sim = FindPackageShare('robot4ws_simulations')
    
    # Declare arguments
    args = [
        DeclareLaunchArgument('use_joystick', default_value='false'),
        DeclareLaunchArgument('use_keyboard', default_value='true'),
        DeclareLaunchArgument('include_terrain_slip_plugin', default_value='false'),
        DeclareLaunchArgument('neural_network_model', default_value='none'),
        DeclareLaunchArgument('world_name', default_value='empty_world.world'),
        DeclareLaunchArgument('gz_sim_args', default_value=''),
        DeclareLaunchArgument('rocker_differential', default_value='false'),
        DeclareLaunchArgument('p310_env', default_value=os.environ.get('PYTHON_VENV', os.path.join(os.path.expanduser('~'), 'p310Venv/bin/python3.10'))),
        DeclareLaunchArgument('add_velodyneHDL32E', default_value='false'),
        DeclareLaunchArgument('lidar_organize_cloud', default_value='false'),
	    DeclareLaunchArgument('include_wheels_terramechanic_model', default_value='false'),
	    DeclareLaunchArgument('terramechanics_config_path', default_value='/home/s250877/archimede_ros2_ws/src/archimede_rover/gz_terramechanics/config'),
    ]

    # Include main gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sim, 'launch', 'archimede_gazebo_simulation_with_kinematics.launch.py'])
        ]),
        launch_arguments={
            'include_terrain_slip_plugin': LaunchConfiguration('include_terrain_slip_plugin'),
            'neural_network_model': LaunchConfiguration('neural_network_model'),
            'rocker_differential': LaunchConfiguration('rocker_differential'),
            'world_name': LaunchConfiguration('world_name'),
            'gz_sim_args': LaunchConfiguration('gz_sim_args'),
            'add_velodyneHDL32E': LaunchConfiguration('add_velodyneHDL32E'),
            'lidar_organize_cloud': LaunchConfiguration('lidar_organize_cloud'),
            'include_wheels_terramechanic_model': LaunchConfiguration('include_wheels_terramechanic_model'),
            'terramechanics_config_path': LaunchConfiguration('terramechanics_config_path'),
        }.items()
    )

    # Joystick teleop (conditional)
    joystick_teleop = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_joystick')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot4ws-teleop-joystick'),
                        'launch',
                        'controller.launch.py'
                    ])
                ]),
                launch_arguments={
                    'env': LaunchConfiguration('p310_env')
                }.items()                
            )
        ]
    )

    # Keyboard teleop (conditional)
    keyboard_teleop = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_keyboard')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot4ws_teleop_keyboard'),
                        'launch',
                        'teleop.launch.py'
                    ])
                ]),
                launch_arguments={
                    'env': LaunchConfiguration('p310_env')
                }.items()
            )
        ]
    )

    return LaunchDescription(
        args + [
            gazebo_sim,
            joystick_teleop,
            keyboard_teleop
        ]
    )