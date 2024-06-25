import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool as BoolMsg

from ros_gz_interfaces.srv import SpawnEntity

from tf2_geometry_msgs import do_transform_pose

from aprs_gz_sim.utils import pose_info, convert_pi_string_to_float, euler_from_quaternion, quaternion_from_euler
from aprs_gz_sim.spawn_params import SpawnParams, PartSpawnParams

import math
import os
from time import sleep
from random import randint

import xml.etree.ElementTree as ET

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Vector3
)

from aprs_interfaces.srv import SpawnPart, SpawnSensor

from ament_index_python.packages import get_package_share_directory



class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"

class PartInfo:
    part_heights = {
        'battery': 0.04,
        'sensor': 0.07,
        'pump': 0.12,
        'regulator': 0.07,
    }

    def __init__(self):
        self.type = None
        self.color = None
        self.rotation = '0'
        self.flipped = False
        self.height = None
        
class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)

class EnvironmentStartup(Node):
    colors = {
        'blue': (0, 0, 168),
        'green': (0, 100, 0),
        'red': (139, 0, 0),
        'purple': (138, 0, 226),
        'orange': (255, 140, 0)   
    }
    
    def __init__(self):
        super().__init__("environment_startup_node")


        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.env_ready = False
        self.environment_ready_publisher = self.create_publisher(BoolMsg, '/aprs_environment_ready', latching_qos)
        
        self.bin_parts_pub_timer = self.create_timer(1.0, self.publish_environment_status)
        
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.part_count = 0
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.spawn_part_client = self.create_client(SpawnPart, "/spawn_part")
        self.spawn_sensor_client = self.create_client(SpawnSensor, "/spawn_sensor")

    def get_sensor_xml(self, file_path, sensor_type, name = "camera_1"):
        xml = ET.fromstring(self.get_sdf(file_path))
        
        xml.find('model').find('link').find('sensor').find('visualize').text = str(False)
        
        ray_sensors = ["break_beam", "proximity", "laser_profiler", "lidar"]
        if sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            # plugin.set('name', str(name + "_ros_plugin"))
            plugin.find('sensor_name').text = name
            plugin.find('frame_name').text = name + "_frame"
        
        cameras = ['rgb_camera', 'rgbd_camera', 'basic_logical_camera', 'advanced_logical_camera']
        if sensor_type in cameras:
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            # plugin.set('name', str(name + "_ros_plugin"))
            plugin.find('camera_name').text = name
            plugin.find('frame_name').text = name + "_frame"

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_sensors(self):
        sensor_type = "advanced_logical_camera"
        xyz = [0, 0, 3]
        name = "advanced_logical_camera"
        
        new_sensor_pose = Pose()
        new_sensor_pose.position.x = float(xyz[0])
        new_sensor_pose.position.y = float(xyz[1])
        new_sensor_pose.position.z = float(xyz[2])
        orientation = quaternion_from_euler(math.pi, math.pi/2, 0.0)
        new_sensor_pose.orientation.x = float(orientation[0])
        new_sensor_pose.orientation.y = float(orientation[1])
        new_sensor_pose.orientation.z = float(orientation[2])
        new_sensor_pose.orientation.w = float(orientation[3])
        
        request = SpawnSensor.Request()
        request.name = name
        request.pose = new_sensor_pose
        
        file_path = os.path.join(get_package_share_directory("ariac_gz_plugins"), "models", sensor_type, "model.sdf")
        
        request.xml = self.get_sensor_xml(file_path, sensor_type, name)
        
        future = self.spawn_sensor_client.call_async(request)
            
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling spawn_sensor service")

        result: SpawnSensor.Response
        result = future.result()

        if not result.success:
            self.get_logger().error("Error calling spawn_sensor service")
    
    def get_sdf(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError:
            return ''
        
        return entity_xml
    
    def get_part_xml(self, p_type, p_color):
        file_path = os.path.join(get_package_share_directory("aprs_gz_sim"), "models", p_type, "model.sdf")
        xml = ET.fromstring(self.get_sdf(file_path))
        
        r, g, b = self.colors[p_color]
        color_string = str(randint(0,255)/255) + " " + str(randint(0,255)/255) + " " + str(randint(0,255)/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'):
            if elem.attrib['name'] == "base":
                elem.find("material").find("ambient").text = color_string
                elem.find("material").find("diffuse").text = color_string

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_parts_for_motoman(self):
        # while True:
        request = SpawnPart.Request()
        
        request.type = ["battery", "pump", "regulator", "sensor"][randint(0,3)]
        request.color = ["blue", "green", "red", "purple", "orange"][randint(0,4)]
        
        new_part_pose = Pose()
        new_part_pose.position.x = 0.0
        new_part_pose.position.y = 0.0
        new_part_pose.position.z = 3.0
        new_part_pose.orientation.x = 0.0
        new_part_pose.orientation.y = 0.0
        new_part_pose.orientation.z = 0.0
        new_part_pose.orientation.w = 0.0
        
        request.pose = new_part_pose
        
        request.xml = self.get_part_xml(request.type, request.color)
        
        future = self.spawn_part_client.call_async(request)
        
        sleep(10)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling spawn_part service")

        result: SpawnPart.Response
        result = future.result()

        if not result.success:
            self.get_logger().error("Error calling spawn_part service")
    
    def publish_environment_status(self):
        msg = BoolMsg()
        msg.data = self.env_ready
        self.environment_ready_publisher.publish(msg)
    
    def environment_ready(self):
        self.env_ready = True
        