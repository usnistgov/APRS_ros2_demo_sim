import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get robot description
    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_lab_robots.urdf.xacro')
    
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')
    
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
    
    # GZ spawn robot
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                '-name', 'aprs_robots',
                '-allow_renaming', 'true'],
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
    
    # robot switcher
    controller_switcher = Node(
        package='aprs_gz_sim',
        executable='combined_controller_switcher_node.py',
        output='screen'
    )
    
    #Joint trajectory controllers
    joint_trajectory_controllers = []
    static_controllers = []
    for robot in ['fanuc', 'franka', 'motoman', 'ur']:
        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_joint_trajectory_controller_spawner',
            arguments=[
                f'{robot}_joint_trajectory_controller', '--inactive'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
        static_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_static_controller_spawner',
            arguments=[
                f'{robot}_static_controller',
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))

    nodes_to_start = [
        robot_state_publisher_node,
        gz_spawn_robot,
        joint_state_broadcaster,
        controller_switcher,
        *joint_trajectory_controllers,
        *static_controllers
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
