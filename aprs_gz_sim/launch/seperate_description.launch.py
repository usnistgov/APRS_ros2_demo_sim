import os
import yaml
import xacro
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

def read_yaml(path):
    with open(path, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to read configuration file")
            return {}  

def launch_setup(context, *args, **kwargs):
    robot_state_publishers = []
    robot_spawners = []
    joint_state_broadcasters = []
    joint_trajectory_controllers = []
    static_controllers = []
    controller_switchers = []

    robots=['fanuc', 'franka', 'motoman', 'ur']
    # robots=["fanuc"]
    sensor_file = os.path.join(get_package_share_directory("aprs_gz_sim"), "config", "sensors.yaml")

    sensor_config = read_yaml(sensor_file)

    for robot in robots:
    # for robot in ["motoman", "fanuc"]:
        urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', f'aprs_{robot}.urdf.xacro')
        
        if robot != "franka":
            xacro_args = {}
            if 'robot_cameras' in sensor_config.keys():
                try:
                    if sensor_config['robot_cameras'][f'{robot}_camera']['active']:
                        xacro_args.update({'camera_active_arg': 'true'})
                        xacro_args.update({'camera_type_arg': sensor_config['robot_cameras'][f'{robot}_camera']['type']})
                except KeyError:
                    rclpy.logging.get_logger('Launch File').error("Unable to parse sensor configuration")
            
            doc = xacro.process_file(urdf, mappings=xacro_args)
        
        else:
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
            # remappings=[
            #     ("joint_states", "/joint_states")
            # ],
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
            name='joint_state_broadcaster_spawner',
            namespace=robot,
            arguments=[
                'joint_state_broadcaster'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
        
        #Joint trajectory controllers    
        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            namespace=robot,
            arguments=[
                'joint_trajectory_controller', '--inactive'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
        
        static_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'static_controller_spawner',
            namespace=robot,
            arguments=[
                'static_controller',
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))
        
        # robot switcher
        controller_switchers.append(Node(
            package='aprs_gz_sim',
            namespace=robot,
            executable='seperate_controller_switcher_node.py',
            output='screen'
        ))

    nodes_to_start = [
        *robot_state_publishers,
        *robot_spawners,
        *joint_state_broadcasters,
        *static_controllers,
        *joint_trajectory_controllers,
        *controller_switchers
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])