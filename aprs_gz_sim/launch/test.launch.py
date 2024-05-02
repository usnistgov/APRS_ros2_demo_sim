import os
import yaml
import xacro
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def launch_setup(context, *args, **kwargs):
    world_path = os.path.join(get_package_share_directory('aprs_gz_sim'), 'worlds', 'lab.sdf')
    
    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ world_path])])
    
    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_ur.urdf.xacro')
    
    ur_description = xacro.process_file(urdf).toxml()
    
    robot_state_publisher = Node(
        name="robot_state_publisher",
        namespace="ur",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        # remappings=[
        #     ('robot_description', 'ur_robot_description')
        # ],
        parameters=[
            {"use_sim_time": True}, 
            {'robot_description': ur_description}
        ]
    )
        
    robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        name="ur_spawner",
        arguments=[
            '-string', ur_description,
            '-name', 'aprs_ur',
            '-allow_renaming', 'true'],
    )
    
    
    nodes_to_start = [
        gz,
        robot_state_publisher,
        robot_spawn,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
