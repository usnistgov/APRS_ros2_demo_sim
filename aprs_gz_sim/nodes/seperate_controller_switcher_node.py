#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

class RobotControllerSwitcher(Node):
    robots_ = ["fanuc", "franka", "motoman", "ur"]
    def __init__(self):
        super().__init__('robot_controller_switcher')
        
        # Create service client to spawn objects into gazebo
        self.controller_switchers = {robot: self.create_client(SwitchController, f'/{robot}/controller_manager/switch_controller') for robot in self.robots_}
        self.requests = {robot: SwitchController.Request() for robot in self.robots_}
        for request in self.requests.values():
            request.strictness = SwitchController.Request.BEST_EFFORT

        self.controllers = {robot: [robot+"_joint_trajectory_controller"] for robot in self.robots_}

        self.recieved_msg = False
        self.robot_health_sub = self.create_subscription(Bool, '/aprs_environment_ready', self.env_ready_cb, 10)
        
        self.robot_states = {robot: True for robot in self.robots_}
    
    def run(self):
        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break
            
            if self.recieved_msg:
                for robot in self.robots_:
                    self.requests[robot].activate_controllers.clear()
                    self.requests[robot].deactivate_controllers.clear()

                    if self.env_ready:
                        if not self.robot_states[robot]:
                            for controller in self.controllers[robot]:
                                self.requests[robot].activate_controllers.append(controller)
                                self.robot_states[robot] = True
                            self.requests[robot].deactivate_controllers.append(f'{robot}_static_controller')
                        if self.robot_states[robot]:
                            for controller in self.controllers[robot]:
                                self.requests[robot].deactivate_controllers.append(controller)
                                self.robot_states[robot] = False
                            self.requests[robot].activate_controllers.append(f'{robot}_static_controller')
                    
                    if self.requests[robot].activate_controllers or self.requests[robot].deactivate_controllers:
                        future = self.controller_switchers[robot].call_async(self.requests[robot])

                        rclpy.spin_until_future_complete(self, future)

                        if not future.result().ok:
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

    