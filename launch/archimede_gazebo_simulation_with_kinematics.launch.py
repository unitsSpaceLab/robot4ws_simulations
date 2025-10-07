#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_sim = FindPackageShare('robot4ws_simulations')
    pkg_kinematics = FindPackageShare('robot4ws_kinematics')
    
    # Declare arguments
    args = [
        DeclareLaunchArgument('include_plugins', default_value='true'),
        DeclareLaunchArgument('include_kinematics', default_value='true'),
        DeclareLaunchArgument('include_terrain_slip_plugin', default_value='false'),
        DeclareLaunchArgument('neural_network_model', default_value='none'),
        DeclareLaunchArgument('include_wheels_terramechanic_model', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='only_walls.sdf'),
        DeclareLaunchArgument('rocker_differential', default_value='true'),
        DeclareLaunchArgument('p310_env', default_value='/home/ros/p310Venv/bin/python3.10'),
        DeclareLaunchArgument('add_velodyneHDL32E', default_value='false'),
        DeclareLaunchArgument('lidar_organize_cloud', default_value='false'),
    ]
    
    # Include basic gazebo launch
    gazebo_basic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sim, 'launch', 'archimede_gazebo_basic.launch.py'])
        ]),
        launch_arguments={
            'include_plugins': LaunchConfiguration('include_plugins'),
            'include_kinematics_plugin': LaunchConfiguration('include_kinematics'),
            'include_terrain_slip_plugin': LaunchConfiguration('include_terrain_slip_plugin'),
            'neural_network_model': LaunchConfiguration('neural_network_model'),
            'include_wheels_terramechanic_model': LaunchConfiguration('include_wheels_terramechanic_model'),
            'rocker_differential': LaunchConfiguration('rocker_differential'),
            'world_name': LaunchConfiguration('world_name'),
            'p310_env': LaunchConfiguration('p310_env'),
            'add_velodyneHDL32E': LaunchConfiguration('add_velodyneHDL32E'),
            'lidar_organize_cloud': LaunchConfiguration('lidar_organize_cloud'),
        }.items()
    )
    
    # Include kinematics launch
    kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_kinematics, 'launch', 'kinematics.launch.py'])
        ]),
        launch_arguments={
            'env': LaunchConfiguration('p310_env')
        }.items()
    )
    
    return LaunchDescription(
        args + [
            gazebo_basic,
            kinematics
        ]
    )