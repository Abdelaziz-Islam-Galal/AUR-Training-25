#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class timer(Node):
    def __init__(self):
        super().__init__("timer")
        # self.get_logger().info("Started timer node")
        self.counter = 10
        if self.counter >= 0:
            self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if self.counter <= 0:
            self.get_logger().info(f"Time is up! {self.counter} have passed!")
        else:
            self.get_logger().info(f"remaining: {self.counter}")
        self.counter -= 1

def main():
    rclpy.init()
    node = timer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
