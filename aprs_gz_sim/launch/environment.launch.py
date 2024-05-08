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
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def launch_setup(context, *args, **kwargs):
    world_path = os.path.join(get_package_share_directory('aprs_gz_sim'), 'worlds', 'lab.sdf')
    
    use_seperate_descriptions = LaunchConfiguration("use_seperate_descriptions")
    
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ world_path])
        ]
    )

    combined_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('aprs_gz_sim'),'launch', 'combined_description.launch.py')]
        ),
        condition=UnlessCondition(use_seperate_descriptions)
    )
    
    seperate_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('aprs_gz_sim'),'launch', 'seperate_description.launch.py')]
        ),
        condition=IfCondition(use_seperate_descriptions)
    )
    
    return [
        gz,
        combined_robots,
        seperate_robots
        ]
    
def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("use_seperate_descriptions", default_value="false", description="use seperate robot descriptions")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
