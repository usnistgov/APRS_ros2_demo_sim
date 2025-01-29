#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

from math import pi
from aprs_interfaces.msg import Trays, Tray, SlotInfo

from aprs_gz_sim.evironment_startup import EnvironmentStartup

from aprs_gz_sim.utils import *

class CloneNode(Node):
    tray_types_ = ["small_gear", "medium_gear", "large_gear", "m2l1_kit", "s2l2_kit"]
    def __init__(self):
        super().__init__('clone_node')
        
        self.spawner_node = EnvironmentStartup()
        self.motoman_trays_spawned = False
        
        motoman_vision_quaternion = quaternion_from_euler(0,0, pi/4)
        motoman_vision_orientation = Quaternion()
        motoman_vision_orientation.w = motoman_vision_quaternion[0]
        motoman_vision_orientation.x = motoman_vision_quaternion[1]
        motoman_vision_orientation.y = motoman_vision_quaternion[2]
        motoman_vision_orientation.z = motoman_vision_quaternion[3]

        self.motoman_vision_pose_ = build_pose(0.0, 0.25, 0.9, motoman_vision_orientation)
        
        motoman_trays_info_sub = self.create_subscription(Trays, '/motoman/table_trays_info', self.update_motoman_trays, 10)
        
        
    
    def update_motoman_trays(self, msg: Trays):
        if self.motoman_trays_spawned:
            return
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        
        for tray in all_trays:
            self.get_logger().info(str(self.motoman_vision_pose_))
            self.get_logger().info(str(tray.tray_pose.pose))
            world_pose = multiply_pose(self.motoman_vision_pose_, tray.tray_pose.pose)
            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(tray.tray_pose.pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append("_".join(slot.name.split("_")[-2:]))
            
            self.spawner_node.spawn_tray(tray_type, tray_color, xyz, rotation, occupied_slots)
        self.motoman_trays_spawned == True

        self.spawner_node.environment_ready()

if __name__ == "__main__":
    rclpy.init()

    clone_node = CloneNode()
    
    try:
        rclpy.spin(clone_node)
    except KeyboardInterrupt:
        clone_node.destroy_node()