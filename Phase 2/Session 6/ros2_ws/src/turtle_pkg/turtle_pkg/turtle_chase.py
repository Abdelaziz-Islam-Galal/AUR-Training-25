#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim_msgs.srv import Spawn, Kill
from turtlesim_msgs.msg import Pose
from std_msgs.msg import Int32
from functools import partial
import random
import math

# Code Functionality:
#     Spawning enemy turtles (always 3 enemies)
#     Detecting collisions
#     kill enemies after collision with user
#     Respawning enemies
#     Publishing the score (enemy death causes score to increment) -> using "ros2 topic echo /score"

# Topics:
#     /spawn (service)
#     /kill (service)
#     /turtle1/pose
#     /enemy1 /pose, /enemy2 /pose, /enemy3 /pose
#     /score -> I will create it

# Functions:
#     player_callback(msg:Pose): receives /turtle1/pose
#     enemy_callback(msg:Pose): receives enemy poses and appends it to a dictionary called enemy_positions:
#         used as -> enemy_positions[name]=msg
#     check_collisions(): timer callback to check for collisions
#     find_distance(pose1: Pose,pose2: Pose)
#     spawn_enemy(name): calls /spawn to create a turtle. -> client
#     kill_enemy(name): calls /kill to remove a turtle. -> client
#     spawn_callback: to make sure spawn occures with no error
#     kill_callback: to make sure spawn occures with no error

class turtle_chase(Node):
    def __init__(self):
        super().__init__("turtle_chase")

        self.user_pos = None
        self.user_pos_sub = self.create_subscription(Pose, '/turtle1/pose', self.player_callback, 10)

        self.score = Int32()
        self.score.data = 0
        self.score_pub = self.create_publisher(Int32, '/score', 10)
        
        self.spawn_enemy("enemy1")
        self.spawn_enemy("enemy2")
        self.spawn_enemy("enemy3")

        self.enemy_positions = {}
        self.enemy1_pos_sub = self.create_subscription(Pose, '/enemy1/pose', partial(self.enemy_callback, 'enemy1'), 10)
        self.enemy2_pos_sub = self.create_subscription(Pose, '/enemy2/pose', partial(self.enemy_callback, 'enemy2'), 10)
        self.enemy3_pos_sub = self.create_subscription(Pose, '/enemy3/pose', partial(self.enemy_callback, 'enemy3'), 10)

        self.create_timer(0.1,self.check_collisions)



    def spawn_enemy(self, name):
        client = self.create_client(Spawn,"/spawn")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting...")

        request = Spawn.Request()
        request.x = random.random() * 11 # ranges from 0 to 11
        request.y = random.random() * 11 # same as x
        request.theta = random.random() * (2*math.pi) # ranges from 0 to 2*pi
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.spawn_callback, name)) # this will call self.callback when service has replied

    def kill_enemy(self, name):
        client = self.create_client(Kill,"/kill")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting...")

        request = Kill.Request()
        request.name = name

        future=client.call_async(request)
        future.add_done_callback(partial(self.kill_callback, name)) # this will call self.callback when service has replied

    def spawn_callback(self, future, name):
        try:
            response = future.result()
            if response.name == name:
                self.get_logger().info(f"spawned {name}")
            else:
                self.get_logger().error(f"error in spawning {name}")
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    
    def kill_callback(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully killed {name}')
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    
    def player_callback(self, msg:Pose):
        self.user_pos = msg

    def enemy_callback(self, name, msg:Pose):
        self.enemy_positions[name] = msg

    def find_distance(self, pose1: Pose, pose2: Pose):
        return math.sqrt((pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2)

    def check_collisions(self):
        if not self.user_pos:
            return
        for name, pose in list(self.enemy_positions.items()):
            dist = self.find_distance(pose, self.user_pos)
            if dist < 0.05:
                self.get_logger().info(f"{name} was hit!")
                self.score.data += 1
                self.score_pub.publish(self.score)
                self.kill_enemy(name)
                self.spawn_enemy(name)

def main():
    rclpy.init()
    node = turtle_chase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()