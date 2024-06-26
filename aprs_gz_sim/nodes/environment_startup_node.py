#!/usr/bin/env python3

from aprs_gz_sim.evironment_startup import EnvironmentStartup
from time import sleep
from random import randint

import rclpy

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    sleep(5)
    
    startup_node.spawn_sensors("advanced_logical_camera", "advanced_logical_camera", [1.0, 0.0, 1.5])
    startup_node.spawn_sensors("advanced_logical_camera", "advanced_logical_camera", [-0.85, 0.5, 1.5])

    part_type = ["battery", "pump", "regulator", "sensor"][randint(0,3)]
    part_color = ["blue", "green", "red", "purple", "orange"][randint(0,4)]
    startup_node.spawn_part(part_type, part_color, [1.0, 0.0, 1.5])
    
    part_type = ["battery", "pump", "regulator", "sensor"][randint(0,3)]
    part_color = ["blue", "green", "red", "purple", "orange"][randint(0,4)]
    startup_node.spawn_part(part_type, part_color, [-0.85, 0.5, 1.5])
    
    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()