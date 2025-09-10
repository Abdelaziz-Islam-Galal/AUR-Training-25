#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from pathlib import Path


class monitor(Node):
    def __init__(self):
        super().__init__("monitor")
        self.get_logger().info("Started monitoring sesnors")
        self.temp = self.create_subscription(Temperature, '/temperature', self.temp_callback, 10)
        self.humid = self.create_subscription(RelativeHumidity, '/humidity', self.humid_callback, 10)
        self.pressure = self.create_subscription(FluidPressure, '/pressure', self.pressure_callback, 10)
        self.print_callback()

    def temp_callback(self, msg):
        return msg.temperature

    def humid_callback(self, msg):
        return msg.relative_humidity * 100

    def pressure_callback(self, msg):
        return msg.fluid_pressure
    
    def print_callback(self):
        self.get_logger().info(f"Temp = {self.temp:.2f} ◦ C, Humidity = {self.humid:.2f} %, Pressure = {self.pressure:.1f} hPa")

        file_name = "monitor_senors.txt"
        file_path = Path(file_name)
        with file_path.open('a') as file:
            file.write(f"Temp = {self.temp:.2f} ◦ C, Humidity = {self.humid:.2f} %, Pressure = {self.pressure:.1f} hPa\n")


def main():
    rclpy.init()
    node = monitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
