#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class QuinticController(Node):
    def __init__(self):
        super().__init__('quintic_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel/test', 10)
        self.timer = self.create_timer(0.05, self.publish_quintic_trajectory)
        self.max_angular_z = 0.07
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def publish_quintic_trajectory(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        # Calculate the quintic trajectory for angular z
        if elapsed_time <= 10.0:
            angular_z = 10 * self.max_angular_z * (elapsed_time / 5.0) ** 3 - 15 * self.max_angular_z * (elapsed_time / 5.0) ** 4 + 6 * self.max_angular_z * (elapsed_time / 5.0) ** 5
        else:
            angular_z = self.max_angular_z

        # Create and publish the Twist message
        twist_msg = Twist()
        # twist_msg.angular.z = angular_z

        # twist_msg.linear.x = 0.12
        # twist_msg.linear.y = 0.04
        twist_msg.angular.z = 0.1
        # print(angular_z)
        print(twist_msg.angular.z)
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    quintic_controller = QuinticController()
    rclpy.spin(quintic_controller)
    quintic_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
