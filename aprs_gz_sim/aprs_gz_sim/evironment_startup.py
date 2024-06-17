import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool as BoolMsg

from gz.sim8.gui  import Spawn

from aprs_gz_sim.utils import pose_info

class SpawnParams:
    def __init__(self, name, file_path=None, xyz=[0,0,0], rpy=[0,0,0], ns='', rf=''):
        self.name = name
        self.xml = ""
        self.file_path = file_path
        self.initial_pose = pose_info(xyz, rpy)
        self.robot_namespace = ns
        self.reference_frame = rf
    
    def get_sdf(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError:
            return ''
        
        return entity_xml
    
    def set_xml_from_file_path(self):
        try:
            f = open(self.file_path, 'r')
            self.xml = f.read()
        except IOError:
            return

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

class EnvironmentStartup(Node):
    def __init__(self):
        super().__init__("environment_startup_node")


        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.env_ready = False
        self.environment_ready_publisher = self.create_publisher(BoolMsg, '/aprs_environment_ready', latching_qos)

        

    def spawn_parts_for_motoman(self):
        pass