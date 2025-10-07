#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot4ws_simulations'),
        'rviz',
        'archimede.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='Archimede_rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([rviz_node])