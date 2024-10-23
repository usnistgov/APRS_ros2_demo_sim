#!/usr/bin/env python3

from aprs_gz_sim.evironment_startup import EnvironmentStartup
from time import sleep
from random import randint, random

import rclpy

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    sleep(2)

    startup_node.spawn_robots()
    
    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()