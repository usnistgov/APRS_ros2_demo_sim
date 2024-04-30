import os
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_motoman.urdf.xacro')
    
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')

    print(robot_description_content)

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    world_path = os.path.join(get_package_share_directory('aprs_description'), 'worlds', 'lab.sdf')
    
    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ world_path])])

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'aprs_robots',
                   '-allow_renaming', 'true'],
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[
            {"use_sim_time": True},
        ],
    )
    
    motoman_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="motoman_controller_spawner",
        arguments=["motoman_joint_trajectory_controller"],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    nodes_to_start = [
        gz,
        robot_state_publisher_node,
        gz_spawn_entity,
        joint_state_broadcaster,
        motoman_controller
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])