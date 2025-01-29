#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

from aprs_interfaces.msg import Trays, Tray, SlotInfo

from aprs_gz_sim.evironment_startup import EnvironmentStartup

from aprs_gz_sim.utils import *

class CloneNode(Node):
    tray_types_ = ["small_gear", "medium_gear", "large_gear", "m2l1_kit", "s2l2_kit"]
    def __init__(self):
        super().__init__('robot_controller_switcher')
        
        self.spawner_node = EnvironmentStartup()
        
        fanuc_trays_info_sub = self.create_subscription(Trays, '/fanuc/table_trays_info', self.update_fanuc_trays, 10)
        
        self.fanuc_vision_pose_ = build_pose(0.0, 0.25, 0.9, quaternion_from_euler(0,0,0))
    
    def update_fanuc_trays(self, msg: Trays):
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays
        
        for tray in all_trays:
            world_pose = multiply_pose(self.fanuc_vision_pose_, tray.tray_pose)
            
            tray_type = self.tray_types_[tray.identifier]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(tray.tray_pose.pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append("_".join(slot.name.split("_")[-2:]))
            
            self.spawner_node.spawn_tray(tray_type, tray_color, xyz, rotation, occupied_slots)