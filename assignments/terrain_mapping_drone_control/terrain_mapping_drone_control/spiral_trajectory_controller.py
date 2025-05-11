#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class SpiralTrajectoryController(Node):
    def __init__(self):
        super().__init__('spiral_trajectory_controller')
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.timer = self.create_timer(0.1, self.publish_target_pose)

        self.radius = 1.0
        self.radius_growth = 0.005
        self.height = 1.5
        self.height_growth = 0.002
        self.theta = 0.0
        self.angular_velocity = 0.2

        self.get_logger().info("Spiral Trajectory Controller Initialized")

    def publish_target_pose(self):
        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)
        z = self.height

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)

        self.theta += self.angular_velocity * 0.1
        self.radius += self.radius_growth
        self.height += self.height_growth

def main(args=None):
    rclpy.init(args=args)
    node = SpiralTrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
