#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

class RobotControllerSwitcher(Node):
    def __init__(self):
        super().__init__('robot_controller_switcher')
        
        # Create service client to spawn objects into gazebo
        self.controller_switcher = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.request = SwitchController.Request()
        self.request.strictness = SwitchController.Request.BEST_EFFORT

        self.controllers = ["joint_trajectory_controller"]

        self.recieved_msg = False
        self.robot_health_sub = self.create_subscription(Bool, '/aprs_environment_ready', self.env_ready_cb, 10)
        
        self.robot_state = False
    
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
                    if not self.robot_state:
                        for controller in self.controllers:
                            self.request.activate_controllers.append(controller)
                            self.robot_state = True
                        self.request.deactivate_controllers.append(f'static_controller')
                    if self.robot_state:
                        for controller in self.controllers:
                            self.request.deactivate_controllers.append(controller)
                            self.robot_state = False
                        self.request.activate_controllers.append(f'static_controller')
                
                if self.request.activate_controllers or self.request.deactivate_controllers:
                    future = self.controller_switcher.call_async(self.request)

                    rclpy.spin_until_future_complete(self, future)

                    if not future.result().ok: # type: ignore
                        self.get_logger().error("Could not switch controllers")
                
                self.recieved_msg = False

                    
    def env_ready_cb(self, msg: Bool):
        self.recieved_msg = True
        self.env_ready = msg.data


if __name__ == "__main__":
    rclpy.init()

    robot_controller_switcher = RobotControllerSwitcher()

    robot_controller_switcher.run()

    robot_controller_switcher.destroy_node()

    