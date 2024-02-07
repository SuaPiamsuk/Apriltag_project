#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class FootPrintNode(Node):
    def __init__(self):
        super().__init__('foot_print')

        self.path = Path()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.count = 0.1
    def timer_callback(self):
        # pose = PoseStamped()
        # self.count = self.count + 0.5
        # # if self.count >= 1.0 :
        # #     self.count = 0.0
        # #     self.path.poses = []

        # self.count = self.count + 0.5

        # pose.pose.position.x = self.count
        # # pose.pose.position.y = 0.1
        # self.path.header.stamp = self.get_clock().now().to_msg()
        # self.path.header.frame_id = 'odom'
        # self.path.poses.append(pose)
        # # self.path.poses.insert
        # self.path_pub.publish(self.path)

        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'odom'
        self.path.poses.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'odom'
        self.path.poses.append(pose)

        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'odom'
        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

    def odom_callback(self, msg):
        # pose = PoseStamped()
        # pose.header = msg.header
        # pose.pose = msg.pose.pose

        # self.count = self.count + 0.5
        # # if self.count >= 1.0 :
        # #     self.count = 0.0
        # #     self.path.poses = []

            

        # pose.pose.position.x = self.count
        # # pose.pose.position.y = 0.1
        # self.path.header = msg.header
        # self.path.poses.append(pose)

        # # pose.pose.position.x = 0.11
        # # pose.pose.position.y = 0.11
        # # self.path.header = msg.header
        # # self.path.poses.append(pose)

        # # pose.pose.position.x = 0.15
        # # pose.pose.position.y = 0.15
        # # self.path.header = msg.header
        # # self.path.poses.append(pose)

        # # pose.pose.position.x = 0.2
        # # self.path.header = msg.header
        # # self.path.poses.append(pose)

        # self.path_pub.publish(self.path)
        pass

def main(args=None):
    rclpy.init(args=args)

    foot_print_node = FootPrintNode()

    rclpy.spin(foot_print_node)

    foot_print_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
