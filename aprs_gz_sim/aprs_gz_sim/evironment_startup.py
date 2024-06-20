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
from os import system
from time import sleep

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Vector3
)

from aprs_interfaces.srv import SpawnPart

# from ariac_msgs.msg import (
#     AssemblyPart,
#     AssemblyTask,
#     BinInfo,
#     BinParts,
#     Challenge,
#     CombinedTask,
#     Condition,
#     ConveyorParts,
#     ConveyorBeltState,
#     DroppedPartChallenge,
#     FaultyPartChallenge,
#     KittingPart,
#     KittingTask,
#     OrderCondition,
#     Order,
#     Part,
#     PartLot,
#     RobotMalfunctionChallenge,
#     SensorBlackoutChallenge,
#     Trial,
# )

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

    def spawn_entity(self, params: SpawnParams, wait=True) -> bool:
        self.get_logger().info("Before wait")
        self.get_logger().info(f'Spawning: {params.name}\n\n\n\n')
        
        service_found = self.spawn_client.wait_for_service(timeout_sec=10)
        
        if not service_found:
            self.get_logger().error("/spawn_entity service not found")
            system("ros2 service list | grep /spawn_entity")
            return False
        
        self.get_logger().info("/spawn_entity service found")

        

        req = SpawnEntity.Request()

        req.entity_factory.name = params.name
        req.entity_factory.pose = params.initial_pose
        req.entity_factory.sdf = params.xml
        req.entity_factory.sdf_filename = params.file_path
        req.entity_factory.relative_to = params.reference_frame

        future = self.spawn_client.call_async(req)

        if wait:
            rclpy.spin_until_future_complete(self, future)
            return future.result().success
        else:
            return True
        
    def parse_part_info(self, part_info):
        part = PartInfo()

        try:
            part.type = part_info['type']
            part.height = PartInfo.part_heights[part.type]
        except KeyError:
            self.get_logger().warn(bcolors.WARNING + "Part type is not specified" + bcolors.ENDC)
            return (False, part)

        try:
            part.color = part_info['color']
        except KeyError:
            self.get_logger().warn(bcolors.WARNING + "Part color is not specified" + bcolors.ENDC)
            return (False, part)

        try:
            part.rotation = str(part_info['rotation'])
        except KeyError:
            pass

        try:
            part.flipped = part_info['flipped']
            if not type(part.flipped) == bool:
                self.get_logger().warn(bcolors.WARNING + "flipped parameter should be either true or false" + bcolors.ENDC)
                part.flipped = False
        except KeyError:
            pass

        if not part.type in PartSpawnParams.part_types:
            self.get_logger().warn(bcolors.WARNING + f"{part_info['type']} is not a valid part type" + bcolors.ENDC)
            return (False, part)

        if not part.color in PartSpawnParams.colors:
            self.get_logger().warn(bcolors.WARNING + f"{part_info['color']} is not a valid part color" + bcolors.ENDC)
            return (False, part)

        return (True, part)

    def spawn_parts_for_motoman(self):
        while True:
            print("TEST")
            request = SpawnPart.Request()
            
            request.type = ["battery", "pump", "regulator", "sensor"][__import__("random").randint(0,3)]
            request.color = "blue"
            
            new_part_pose = Pose()
            new_part_pose.position.x = 0.0
            new_part_pose.position.y = 0.0
            new_part_pose.position.z = 3.0
            new_part_pose.orientation.x = 0.0
            new_part_pose.orientation.y = 0.0
            new_part_pose.orientation.z = 0.0
            new_part_pose.orientation.w = 0.0
            
            request.pose = new_part_pose
            
            future = self.spawn_part_client.call_async(request)
            
            sleep(1)
            # while not future.done():
            #     sleep(0.1)
            #     print("IN LOOP")
            #     pass

            # if not future.done():
            #     raise Error("Timeout reached when calling spawn_part service")

            # result: SpawnPart.Response
            # result = future.result()

            # if not result.success:
            #     self.get_logger().error("Error calling spawn_part service")
    
    def publish_environment_status(self):
        msg = BoolMsg()
        msg.data = self.env_ready
        self.environment_ready_publisher.publish(msg)
    
    def environment_ready(self):
        self.env_ready = True
        