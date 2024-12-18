#!/usr/bin/env python3

from aprs_gz_sim.evironment_startup import EnvironmentStartup
from time import sleep
from random import randint, random

import rclpy

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    sleep(10)
    
    startup_node.spawn_sensors("advanced_logical_camera", "advanced_logical_camera", [0.0, 0.0, 5.0])

    part_type = "medium"
    part_color = "green"
    startup_node.spawn_gear(part_type, part_color, [0.0, 0.25, 5])
    
    # part_type = "regulator"
    # part_color = "green"
    # startup_node.spawn_part(part_type, part_color, [1.1, -1.375, 1.0])
    
    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()