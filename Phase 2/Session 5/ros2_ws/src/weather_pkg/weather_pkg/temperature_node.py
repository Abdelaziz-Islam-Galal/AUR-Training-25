#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random

class temp_sensor(Node):
    def __init__(self):
        super().__init__("temp")
        self.get_logger().info("Started measuring temprature")
        self.publisher_node = self.create_publisher(Temperature, '/temperature', 10)
        self.counter = 0
        self.create_timer(1,  self.rand_temp_callback)

    def rand_temp_callback(self):
        msg = Temperature()
        msg.temperature = random.random() * 45 # ranges from 0 to 45 Degrees Celcius
        msg.variance = 0 # variance is unknown
        self.publisher_node.publish(msg)


def main():
    rclpy.init()
    node = temp_sensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
