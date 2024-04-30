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
    # urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_lab_robots.urdf.xacro')
    
    # doc = xacro.process_file(urdf)

    # robot_description_content = doc.toprettyxml(indent='  ')

    # print(robot_description_content)

    # robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[{"use_sim_time": True}, robot_description],
    # )

    world_path = os.path.join(get_package_share_directory('aprs_description'), 'worlds', 'lab.sdf')
    
    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ world_path])])

    # gz_spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=['-string', doc.toxml(),
    #                '-name', 'aprs_robots',
    #                '-allow_renaming', 'true'],
    # )
    
    # joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="joint_state_broadcaster_spawner",
    #     arguments=["joint_state_broadcaster"],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )
    
    # ur_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="ur_controller_spawner",
    #     arguments=["ur_joint_trajectory_controller"],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )
    
    # fanuc_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="fanuc_controller_spawner",
    #     arguments=["fanuc_joint_trajectory_controller"],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )
    
    # franka_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="franka_controller_spawner",
    #     arguments=["franka_joint_trajectory_controller"],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )
    
    # motoman_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="motoman_controller_spawner",
    #     arguments=["motoman_joint_trajectory_controller"],
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )

    nodes_to_start = [
        gz,
        # robot_state_publisher_node,
        # gz_spawn_entity,
        # joint_state_broadcaster,
        # ur_controller,
        # fanuc_controller,
        # franka_controller,
        # motoman_controller
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # declared_arguments.append(
    #     DeclareLaunchArgument("trial_name", default_value="kitting", description="name of trial")
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument("competitor_pkg", default_value="ariac_gazebo", description="name of competitor package"))

    # declared_arguments.append(
    #     DeclareLaunchArgument("sensor_config", default_value="sensors", description="name of user configuration file")
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument("dev_mode", default_value="false", description="run simulation in dev mode")
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument("record_state", default_value="false", description="record a gazebo state.log")
    # )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
