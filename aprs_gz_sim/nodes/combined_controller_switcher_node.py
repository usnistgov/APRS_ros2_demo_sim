#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool


class RobotControllerSwitcher(Node):
    def __init__(self):
        super().__init__('robot_controller_switcher')
        
        # Create service client to spawn objects into gazebo
        self.controller_switcher = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.request = SwitchController.Request()
        self.request.strictness = SwitchController.Request.BEST_EFFORT

        self.fanuc_controllers = ["fanuc_joint_trajectory_controller"]
        self.franka_controllers = ["franka_joint_trajectory_controller"]
        self.motoman_controllers = ["motoman_joint_trajectory_controller"]
        self.ur_controllers = ["ur_joint_trajectory_controller"]

        self.recieved_msg = False
        self.robot_health_sub = self.create_subscription(Bool, '/aprs_environment_ready', self.env_ready_cb, 10)
        
        self.fanuc_state = False
        self.franka_state = False
        self.motoman_state = False
        self.ur_state = False
        
        self.env_ready = False
    
    def run(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break

            if self.recieved_msg:
                self.request.activate_controllers.clear()
                self.request.deactivate_controllers.clear()

                if self.env_ready:
                    if not self.fanuc_state:
                        for controller in self.fanuc_controllers:
                            self.request.activate_controllers.append(controller)
                            self.fanuc_state = True
                        self.request.deactivate_controllers.append("fanuc_static_controller")
                
                    if not self.franka_state:
                        for controller in self.franka_controllers:
                            self.request.activate_controllers.append(controller)
                            self.franka_state = True
                        self.request.deactivate_controllers.append("franka_static_controller")
                    
                    if not self.motoman_state:
                        for controller in self.motoman_controllers:
                            self.request.activate_controllers.append(controller)
                            self.motoman_state = True
                        self.request.deactivate_controllers.append("motoman_static_controller")
                    
                    if not self.ur_state:
                        for controller in self.ur_controllers:
                            self.request.activate_controllers.append(controller)
                            self.ur_state = True
                        self.request.deactivate_controllers.append("ur_static_controller")
                
                if not self.env_ready:
                    if self.fanuc_state:
                        for controller in self.fanuc_controllers:
                            self.request.deactivate_controllers.append(controller)
                            self.fanuc_state = False
                        self.request.activate_controllers.append("fanuc_static_controller")
                    
                    if self.franka_state:
                        for controller in self.franka_controllers:
                            self.request.deactivate_controllers.append(controller)
                            self.franka_state = False
                        self.request.activate_controllers.append("franka_static_controller")
                    
                    if self.motoman_state:
                        for controller in self.motoman_controllers:
                            self.request.deactivate_controllers.append(controller)
                            self.motoman_state = False
                        self.request.activate_controllers.append("motoman_static_controller")
                    
                    if self.ur_state:
                        for controller in self.ur_controllers:
                            self.request.deactivate_controllers.append(controller)
                            self.ur_state = False
                        self.request.activate_controllers.append("ur_static_controller")

                if self.request.activate_controllers or self.request.deactivate_controllers:
                    future = self.controller_switcher.call_async(self.request)

                    rclpy.spin_until_future_complete(self, future)

                    if not future.result().ok:
                        self.get_logger().error("Could not switch controllers")
                
                self.recieved_msg = False

                    
    def env_ready_cb(self, msg: Bool):
        self.get_logger().info("Recieved msg")
        self.recieved_msg = True
        self.env_ready = msg.data


if __name__ == "__main__":
    rclpy.init()

    robot_controller_switcher = RobotControllerSwitcher()

    robot_controller_switcher.run()

    robot_controller_switcher.destroy_node()

    