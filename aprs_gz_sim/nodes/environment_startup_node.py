#!/usr/bin/env python3

from aprs_gz_sim.evironment_startup import EnvironmentStartup
from time import sleep

import rclpy

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    print("Stuff loaded")

    sleep(5)

    startup_node.spawn_parts_for_motoman()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()