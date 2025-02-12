import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    world_path = os.path.join(get_package_share_directory('aprs_gz_sim'), 'worlds', 'test.sdf')
    
    use_seperate_descriptions = LaunchConfiguration("use_seperate_descriptions")
    
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v4 '+ world_path])
        ]
    )

    bridge_params = os.path.join(get_package_share_directory("aprs_gz_sim"),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    return [
        gz,
        ros_gz_bridge
        ]
    
def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("use_seperate_descriptions", default_value="false", description="use seperate robot descriptions")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
