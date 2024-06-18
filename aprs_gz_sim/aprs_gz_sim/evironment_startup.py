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

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Vector3
)

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
        _, part = self.parse_part_info({'type': 'battery', 'color': 'blue'})

        part_name = part.type + "_" + part.color + "_b" + str(self.part_count).zfill(2)
        self.part_count += 1

        if part.flipped:
            roll = math.pi
        else:
            roll = 0

        yaw = convert_pi_string_to_float(part.rotation)

        q = quaternion_from_euler(roll, 0, yaw)
        rel_pose = Pose()
        rel_pose.position.x = 0.0
        rel_pose.position.y = 0.0

        if part.flipped:
            rel_pose.position.z = part.height

        rel_pose.orientation.w = q[0]
        rel_pose.orientation.x = q[1]
        rel_pose.orientation.y = q[2]
        rel_pose.orientation.z = q[3]
        
        world_pose = do_transform_pose(rel_pose, self.tf_buffer.lookup_transform("world", "world", rclpy.time.Time()))

        xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
        rpy = euler_from_quaternion(world_pose.orientation)

        params = PartSpawnParams(part_name, part.type, part.color, xyz=xyz, rpy=rpy)

        self.spawn_entity(params, wait=True)
    
    def publish_environment_status(self):
        msg = BoolMsg()
        msg.data = self.env_ready
        self.environment_ready_publisher.publish(msg)
    
    def environment_ready(self):
        self.env_ready = True
        