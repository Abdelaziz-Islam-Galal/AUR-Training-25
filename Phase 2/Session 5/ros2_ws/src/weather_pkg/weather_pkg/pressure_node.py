#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
import random

class pressure_sensor(Node):
    def __init__(self):
        super().__init__("pressure")
        self.get_logger().info("Started measuring pressure")
        self.publisher_node = self.create_publisher(FluidPressure, '/pressure', 10)
        self.counter = 0
        self.create_timer(1,  self.rand_pressure_callback)

    def rand_pressure_callback(self):
        msg = FluidPressure()
        msg.fluid_pressure = random.randint(900, 1100) + random.random() # ranges from 900 to 1101
        msg.variance = 0 # variance is unknown
        self.publisher_node.publish(msg)


def main():
    rclpy.init()
    node = pressure_sensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
