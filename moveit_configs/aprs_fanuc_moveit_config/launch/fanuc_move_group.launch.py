from launch import LaunchDescription
from launch.actions import OpaqueFunction

import os

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    urdf = os.path.join(get_package_share_directory("aprs_description"), "urdf/aprs_fanuc.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("aprs_fanuc", package_name="aprs_fanuc_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/aprs_fanuc.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace="fanuc",
        output="screen",
        # remappings=[
        #     ('/joint_states', '/fanuc/joint_states')             
        # ],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )   

    nodes_to_start = [
        move_group_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])