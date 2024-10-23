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
    
    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_lab_robots.urdf.xacro')

    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')
    
    use_seperate_descriptions = LaunchConfiguration("use_seperate_descriptions")
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            'world': world_path,
        }.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description_content}
        ],
    )
    
    spawn_part_node = Node(
        package='aprs_gz_sim',
        executable='spawn_part'
    )
    
    
    environment_startup_node = Node(
        package='aprs_gz_sim',
        executable='environment_startup_node.py',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content}
        ]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description_content}
        ],
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[
            {'use_sim_time': True},
        ],
    )
    
    #Joint trajectory controllers
    joint_trajectory_controllers = []
    for robot in ['fanuc', 'franka', 'motoman', 'ur']:
        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_controller_spawner',
            arguments=[
                f'{robot}_joint_trajectory_controller'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
    
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
        gazebo,
        # combined_robots,
        # seperate_robots,
        # spawn_part_node,
        environment_startup_node,
        robot_state_publisher_node,
        joint_state_broadcaster,
        *joint_trajectory_controllers
        ]
    
def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("use_seperate_descriptions", default_value="false", description="use seperate robot descriptions")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
