#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses

class publisher(Node):
    def __init__(self):
        super().__init__("turtle2_WASD")
        self.turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True) 

        self.timer = self.create_timer(0.1, self.publish_commands)

    def publish_commands(self):
        input_key = self.get_input()
        
        msg2 = self.move_turtle2(input_key)
        msg1 = self.move_turtle1(input_key)
        
        self.turtle2.publish(msg2)
        self.turtle1.publish(msg1)
    
    def move_turtle2(self, input_key):
        msg = Twist()
        if input_key == 'w':
            msg.linear.x = 2.0
        elif input_key == 'a':
            msg.angular.z = 2.0
        elif input_key == 's':
            msg.linear.x = -2.0
        elif input_key == 'd':
            msg.angular.z = -2.0

        return msg
    
    def move_turtle1(self, input_key):
        msg = Twist()
        if input_key == 'up':
            msg.linear.x = 2.0
        elif input_key == 'left':
            msg.angular.z = 2.0
        elif input_key == 'down':
            msg.linear.x = -2.0
        elif input_key == 'right':
            msg.angular.z = -2.0

        return msg


    def get_input(self):
        key = self.stdscr.getch()
        
        if key != -1:  # -1 means no key is pressed
            if key == ord('w'):
                return 'w'
            elif key == ord('a'):
                return 'a'
            elif key == ord('s'):
                return 's'
            elif key == ord('d'):
                return 'd'
            elif key == curses.KEY_UP:
                return 'up'
            elif key == curses.KEY_LEFT:
                return 'left'
            elif key == curses.KEY_DOWN:
                return 'down'
            elif key == curses.KEY_RIGHT:
                return 'right'
        
        return None
    
    def cleanup(self):
        curses.nocbreak()
        self.stdscr.keypad(False)
        curses.echo()
        curses.endwin()

def main():
    rclpy.init()
    node = publisher()
    rclpy.spin(node)
    node.cleanup() # for curses (closing it)
    node.destroy_node()
    rclpy.shutdown()
                
