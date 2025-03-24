#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        # self.get_logger().info("("+ str(msg.x)+", "+str(msg.y)+")")
        self.get_logger().info(f"X: {msg.x}, Y: {msg.y}, Theta: {msg.theta}")
        
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()