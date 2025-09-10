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
        self.temp_sub = self.create_subscription(Temperature, '/temperature', self.temp_callback, 10)
        self.humid_sub = self.create_subscription(RelativeHumidity, '/humidity', self.humid_callback, 10)
        self.pressure_sub = self.create_subscription(FluidPressure, '/pressure', self.pressure_callback, 10)
        self.timer = self.create_timer(1.0, self.print_callback)
        self.file_name = "monitor_senors.txt"
        self.file_path = Path(self.file_name)

    def temp_callback(self, msg):
        self.temp = msg.temperature

    def humid_callback(self, msg):
        self.humid = msg.relative_humidity * 100

    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure
    
    def print_callback(self):
        self.get_logger().info(f"Temp = {self.temp:.2f} ◦ C, Humidity = {self.humid:.2f} %, Pressure = {self.pressure:.1f} hPa")

        with self.file_path.open('a') as file:
            file.write(f"Temp = {self.temp:.2f} ◦ C, Humidity = {self.humid:.2f} %, Pressure = {self.pressure:.1f} hPa\n")


def main():
    rclpy.init()
    node = monitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
                
