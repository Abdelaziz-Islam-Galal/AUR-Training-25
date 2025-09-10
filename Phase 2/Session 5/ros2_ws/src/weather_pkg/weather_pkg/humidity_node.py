#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RelativeHumidity
import random

class humid_sensor(Node):
    def __init__(self):
        super().__init__("humid")
        self.get_logger().info("Started measuring Humidity")
        self.publisher_node = self.create_publisher(RelativeHumidity, '/humidity', 10)
        self.counter = 0
        self.create_timer(1,  self.rand_humid_callback)

    def rand_humid_callback(self):
        msg = RelativeHumidity()
        msg.relative_humidity = random.random() # ranges from 0 to 1 (percentage)
        msg.variance = 0 # variance is unknown
        self.publisher_node.publish(msg)


def main():
    rclpy.init()
    node = humid_sensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
