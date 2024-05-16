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
    robot_state_publishers = []
    robot_spawners = []
    joint_state_broadcasters = []
    joint_trajectory_controllers = []
    # for robot in ['fanuc', 'franka', 'motoman', 'ur']:
    for robot in ["fanuc","franka"]:
        urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', f'aprs_{robot}.urdf.xacro')
        
        doc = xacro.process_file(urdf)

        robot_description_content = doc.toprettyxml(indent='  ')
        
        # Robot state publisher
        robot_state_publisher_params = {'use_sim_time': True,
                                        'robot_description': robot_description_content}
        robot_state_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            namespace=robot,
            remappings=[
                ("joint_states", "/joint_states")
            ],
            parameters=[
                robot_state_publisher_params
            ],
        ))
        
        # GZ spawn robot
        robot_spawners.append(Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            # name=f'{robot}_ros_gz_sim',
            arguments=[
                    '-topic', f'{robot}/robot_description',        
                    '-name', f'aprs_{robot}',
                    '-allow_renaming', 'true']
        ))
        
        # Joint state broadcaster
        joint_state_broadcasters.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_joint_state_broadcaster_spawner',
            arguments=[
                f'{robot}_joint_state_broadcaster',
                '-c', f'{robot}_controller_manager'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
        
        #Joint trajectory controllers    
        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_controller_spawner',
            arguments=[
                f'{robot}_joint_trajectory_controller', 
                '-c', f'{robot}_controller_manager'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))

    nodes_to_start = [
        *robot_state_publishers,
        *robot_spawners,
        *joint_state_broadcasters,
        *joint_trajectory_controllers
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
