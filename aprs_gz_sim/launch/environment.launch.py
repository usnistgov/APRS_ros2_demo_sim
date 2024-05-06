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
    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_lab_robots.urdf.xacro')
    
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')

    # print(robot_description_content)

    robot_description = {"robot_description": robot_description_content}

    

    world_path = os.path.join(get_package_share_directory('aprs_gz_sim'), 'worlds', 'lab.sdf')
    
    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ world_path])])

    use_seperate_descriptions = str(LaunchConfiguration("use_seperate_descriptions").perform(context)).lower() == "true"
    
    if use_seperate_descriptions:
        robot_names = ["fanuc"]
        robot_urdf_docs = {name:xacro.process_file(os.path.join(get_package_share_directory('aprs_description'), 'urdf', f'aprs_{name}.urdf.xacro')) for name in robot_names}
        robot_descriptions = {name:{"robot_description":robot_urdf_docs[name].toprettyxml(indent='  ')} for name in robot_names}
        
        robot_state_publishers = []
        for robot_name in robot_names:
            robot_state_publishers.append(Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                # namespace=robot_name,
                remappings=[
                    ('robot_description', f'/{robot_name}_robot_description'),
                    # (f'/{robot_name}/joint_states', f'/{robot_name}_joint_states')             
                ],
                output="both",
                parameters=[{"use_sim_time": True}, robot_descriptions[robot_name]],
            ))

        gz_spawners = []
        for robot_name in robot_names:
            gz_spawners.append(Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                # namespace=f"{robot_name}",
                name=f"{robot_name}_spawner",
                arguments=['-string', robot_urdf_docs[robot_name].toxml(),
                        '-name', f'aprs_{robot_name}',
                        '-allow_renaming', 'true'],
            ))
        
        joint_state_broadcasters = []
        for robot_name in robot_names:
            joint_state_broadcasters.append(Node(
                package="controller_manager",
                executable="spawner",
                name=f"{robot_name}_joint_state_broadcaster_spawner",
                arguments=[
                    "joint_state_broadcaster",
                    # "-c", f"/{robot_name}/controller_manager"
                ],
                parameters=[
                    {"use_sim_time": True},
                ],
            ))
        
        controller_spawner_nodes = []
        for robot_name in robot_names:
            controller_spawner_nodes.append(Node(
                package="controller_manager",
                executable="spawner",
                name=f"{robot_name}_controller_spawner",
                arguments=[
                    "joint_trajectory_controller", 
                    # "-c", f"/{robot_name}/controller_manager"
                ],
                parameters=[
                    {"use_sim_time": True},
                ],
            ))
            
        move_groups = []
        for robot_name in robot_names:
            move_groups.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare(f"aprs_{robot_name}_moveit_config"), "/launch", f"/{robot_name}_move_group.launch.py"]
            )))

        nodes_to_start = [
            gz,
            *controller_spawner_nodes,
            *robot_state_publishers,
            *gz_spawners,
            *joint_state_broadcasters,
            # *move_groups
        ]
    else:
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"use_sim_time": True}, robot_description],
        )
        
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
        
        ur_controller = Node(
            package="controller_manager",
            executable="spawner",
            name="ur_controller_spawner",
            arguments=["ur_joint_trajectory_controller"],
            parameters=[
                {"use_sim_time": True},
            ],
        )
        
        fanuc_controller = Node(
            package="controller_manager",
            executable="spawner",
            name="fanuc_controller_spawner",
            arguments=["fanuc_joint_trajectory_controller"],
            parameters=[
                {"use_sim_time": True},
            ],
        )
        
        franka_controller = Node(
            package="controller_manager",
            executable="spawner",
            name="franka_controller_spawner",
            arguments=["franka_joint_trajectory_controller"],
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
        
        move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("aprs_moveit_config"), "/launch", "/move_group.launch.py"]
            )
        )

        nodes_to_start = [
            gz,
            robot_state_publisher_node,
            gz_spawn_entity,
            joint_state_broadcaster,
            ur_controller,
            fanuc_controller,
            franka_controller,
            motoman_controller,
            move_group
        ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("use_seperate_descriptions", default_value="false", description="use seperate robot descriptions")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
