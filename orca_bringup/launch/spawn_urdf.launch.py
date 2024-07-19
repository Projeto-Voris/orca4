"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file."""


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = PathJoinSubstitution([FindPackageShare('orca_description'), 'worlds', 'sand.world'])

    urdf = PathJoinSubstitution([FindPackageShare('orca_description'), 'urdf', 'bluerov2.xacro'])

    # xml = open(urdf, 'r').read()
    #
    # xml = xml.replace('"', '\\"')
    #
    # swpan_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', world],
            output='screen'),

        Node(
            package='orca_description',
            executable='spawn_entity.py',
            arguments=['-entity', urdf,
                       '-topic', 'robot_description',
                       '-x', '1',
                       '-y', '1',
                       '-z', '0',
                       '-Y', '0'], #yaw
            output='screen')
    ])