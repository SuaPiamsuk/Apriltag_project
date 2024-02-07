#!/usr/bin/env python3


import threading
import typing

import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import numpy.typing as npt
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.subscription import Subscription
from rclpy.node import Node


class Example_Node(Node):
    """Example Node for showing how to use matplotlib within ros 2 node
    
    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        x: x values for matplotlib
        y: y values for matplotlib
        lock: lock for threading
        _sub: Subscriber for node
    """

    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        self.ax.grid()
        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        # create initial values to plot
        self.x = [i for i in range(5)]
        self.width = [0.5 for i in range(5)]
        # create subscriber
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sub: Subscription = self.create_subscription(
            PoseArray, "points", self._callback, 10, callback_group=self.cbg
        )

        self.x = 0
        self.y = 0
        
        self.x_values = []
        self.y_values = []

        self.docking_x = 0
        self.docking_y = 0

        self.mid_point_x = 0.0
        self.mid_point_y = 0.0

    def _callback(self, msg):
        """Callback for subscriber
        
        Args:
            msg: message from subscriber
                Message format
                ----------------
                int32 num
        """
        # lock thread
        with self._lock:
            # # update values
            # number: int = msg.poses[0].position.y
            # # print(msg.poses[0])
            # # self.get_logger().info('test "%s"'% msg.poses[0])
            # # self.get_logger().)
            # self.x.append(msg.poses[0].position.x)
            # self.width.append(0.5)

            self.x = msg.poses[0].position.x
            self.y = msg.poses[0].position.y

            points = np.array([[msg.poses[0].position.x, msg.poses[0].position.y + 1], [msg.poses[2].position.x, msg.poses[2].position.y - 1]])

            # ทำการทำฟิตเส้นตรงโดยใช้ np.polyfit
            # กรณีทำฟิตเส้นตรงเพื่อหาสมการของเส้นตรง
            coefficients = np.polyfit(points[:, 0], points[:, 1], 1)

            # สร้างฟังก์ชันเส้นตรงจากค่าพหุคณะที่ได้
            line_function = np.poly1d(coefficients)

            # สร้างข้อมูลสำหรับพล็อตเส้นตรง
            self.x_values = np.linspace(min(points[:, 0]), max(points[:, 0]), 100)
            self.y_values = line_function(self.x_values)


            x_min  = msg.poses[0].position.x
            x_max  = msg.poses[2].position.x
            y_min = line_function(x_min)
            y_max = line_function(x_max)

            slope = (y_max - y_min) / (x_max - x_min)

            slope = -1/slope

            self.mid_point_x  = (x_max + x_min)/2.0
            self.mid_point_y  = (y_max + y_min)/2.0

            self.docking_y = -1.0

            self.docking_x = ((self.docking_y - self.mid_point_y) / slope) + self.mid_point_x


            # plt.plot([mid_point_x, x], [mid_point_y, y], marker='o', label='pre_dock')

    def plt_func(self, _):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            # x = np.array(self.x)
            # y = np.array(self.width)
            # self.ax.barh(y, 0.5, left=x, color="red")
            # self.ax.scatter(self.x, self.y, label='ddd')
            self.ax.plot(self.y_values, self.x_values, label='station')
            plt.plot([self.mid_point_y, self.docking_y], [self.mid_point_x, self.docking_x], marker='o', label='pre_dock')

            self.ax.axis('square')
            # self.ax.grid()
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-5.5, 2)

            return self.ax

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Example_Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt()


if __name__ == "__main__":
    main()


# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String
# from apriltag_msgs.msg import AprilTagDetectionArray

# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

# #add frame
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped

# from std_msgs.msg import Float32


# from geometry_msgs.msg import Twist

# import math
# from math import atan2
# from math import sin, cos

# import numpy as np

# import matplotlib.pyplot as plt
# # from matplotlib.animation import FuncAnimation

# import matplotlib.animation as anim
# from rclpy.subscription import Subscription
# import threading
# from geometry_msgs.msg import PoseArray

# class docking(Node):

#     def __init__(self):
#         super().__init__('docking')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         timer_period = 0.1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#         self.publisher_float = self.create_publisher(Float32, 'random_data', 10)

#         self.pose_stamped_publisher = self.create_publisher(PoseStamped, 'pose_stamped_topic', 10)

#         # self.subscription = self.create_subscription(
#         #     AprilTagDetectionArray,
#         #     'apriltag_detections',
#         #     self.listener_callback,
#         #     10)
#         # self.subscription  # prevent unused variable warning

#         # self.odom_sub = self.create_subscription(
#         #     Odometry,
#         #     '/odom',
#         #     self.odom_callback,
#         #     10
#         # )
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # self.tf_broadcaster = TransformBroadcaster(self)

#         self.fig, self.ax = plt.subplots()
#         self._lock = threading.Lock()
#         # self.line, = self.ax.plot([], [], 'b-', label='Line')
#         # self.points, = self.ax.plot([], [], 'ro', label='Points')
#         # self.ax.legend()
#         # self.ax.set_xlim(0, 10)
#         # self.ax.set_ylim(0, 10)
#         self.point_path, = self.ax.plot([], [], 'o',color='orange', markersize=2, label ='Path')
#         # plt.show(block=True)


#         # Initialize figure and axes and save to class
#         # self.fig, self.ax = plt.subplots()
#         # create Thread lock to prevent multiaccess threading errors
#         # self._lock = threading.Lock()
#         self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
#         # self._sub: Subscription = self.create_subscription(
#         #     Test, "test", self._callback, 10, callback_group=self.cbg
#         # )
#         self.count = 0.0

#         self._sub: Subscription = self.create_subscription(
#             PoseArray, "points", self._callback, 10
#         )

#     def _callback(self, msg):
#         self.get_logger().info('test "%s"'% msg.poses[0].position.y)

#     def timer_callback(self):
#         try:
#             t1 = self.tf_buffer.lookup_transform(
#                 # 'base_footprint',
#                 'camera_link',
#                 'dock_frame1',
#                 rclpy.time.Time())
            
#         # except TransformException as ex:
#         #     t1=[]
#         #     self.get_logger().info(
#         #         f'Could not transform_dock1')
#             # return
#         except:
#             t1=[]            
        
#         try:
#             t2 = self.tf_buffer.lookup_transform(
#                 # 'base_footprint',
#                 'camera_link',
#                 'dock_frame2',
#                 rclpy.time.Time())
#         except:
#             t2=[]

#         try:
#             t3 = self.tf_buffer.lookup_transform(
#                 # 'base_footprint',
#                 'camera_link',
#                 'dock_frame3',
#                 rclpy.time.Time())
#         except:
#             t3=[]


#         try:
#             # self.get_logger().info('a: "%s"' % 'slope')
#             x = [t1.transform.translation.x, t2.transform.translation.x, t3.transform.translation.x]
#             y = [t1.transform.translation.y, t2.transform.translation.y, t3.transform.translation.y]

#             coefficients = np.polyfit(x, y, 1)
#             line_function = np.poly1d(coefficients)

#             y_values = (t1.transform.translation.x+t3.transform.translation.x)/2.0
#             y_values = line_function(y_values)

#             # y_values = line_function(t2.transform.translation.x)

#             x_min = t1.transform.translation.x
#             x_max = t3.transform.translation.x
#             y_min = line_function(x_min)
#             y_max = line_function(x_max)

#             # x_min_backup = t1.transform.translation.x
#             # x_max_backup = t3.transform.translation.x

#             # x_min = round(x_min_backup, 4)
#             # x_max = round(x_max_backup ,4)
#             # y_min = round(line_function(x_min),4)
#             # y_max = round(line_function(x_max),4)

#             # x_min = round(x_min,3) 
#             # x_max = round(x_max,3)
#             # y_min = round(y_min,3)
#             # y_max = round(y_max,3)


#             # self.get_logger().info('y_max: "%s"' % y_max)
#             # self.get_logger().info('y_min: "%s"' % y_min)
#             slope = round((y_max - y_min),5) / round((x_max - x_min),5)
#             # self.get_logger().info('a: "%s"' % slope)

#             msg = Float32()
#             msg.data = y_max - y_min
#             self.publisher_float.publish(msg)

#             slope = round(slope,5)
#             slope = -1/slope
#             # self.get_logger().info('b: "%s"' % slope)
#             slope = round(slope,5)

#             # msg.data = slope
#             # self.publisher_float.publish(msg)

#             mid_point_x  = (x_max + x_min)/2.0
#             mid_point_y  = (y_max + y_min)/2.0

#             # y = -100

#             # x = ((y - mid_point_y) / slope) + mid_point_x

#             xx =  0.4*t1.transform.translation.x

#             yy = mid_point_y - slope*(mid_point_x-xx)

#             cout = slope*(mid_point_x-xx)

#             # self.get_logger().info('x: "%s"' % mid_point_y)
#             # self.get_logger().info('x1: "%s"' % mid_point_x)
#             # self.get_logger().info('x2: "%s"' % slope)
#             # self.get_logger().info('y: "%s"' % yy)



#             tf = TransformStamped()
#             tf.header.stamp = self.get_clock().now().to_msg()
#             tf.header.frame_id = 'camera_link'
#             tf.child_frame_id = 'docking'
#             tf.transform.translation.x = xx
#             tf.transform.translation.y = yy
#             tf.transform.translation.z = 0.0
#             tf.transform.rotation.x = t2.transform.rotation.x
#             tf.transform.rotation.y = t2.transform.rotation.y
#             tf.transform.rotation.z = t2.transform.rotation.z
#             tf.transform.rotation.w = t2.transform.rotation.w

#             self.tf_broadcaster.sendTransform(tf)

#             # self.tf_broadcaster = TransformBroadcaster(self)


#             # plt.plot([1,2], [1,2], label='station')
#             # self.point_path.set_data([1],[1])
#             # plt.show(block=False)

#             # # lock thread
#             # with self._lock:
#             #     # update values
#             #     number: int = msg.num
#             #     self.x.append(number)
#             #     self.width.append(0.5)

            
#         except:
#             pass

#         self.count = self.count + 0.01
#         # self.get_logger().info('a: "%s"' % 'slope')

#     def plt_func(self, _):
#         """Function for for adding data to axis.

#         Args:
#             _ : Dummy variable that is required for matplotlib animation.
        
#         Returns:
#             Axes object for matplotlib
#         """
#         # lock thread
#         with self._lock:
#             # x = np.array(self.x)
#             # y = np.array(self.width)
#             # self.ax.barh(1, 0.5, left=1, color="red")
#             self.ax.plot([self.count],[2], 'ro', markersize=2,  label ='Lidar')

#             return self.ax

#     def _plt(self):
#         """Function for initializing and showing matplotlib animation."""
#         self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
#         plt.show()


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = docking()
#     # plt.show(block=True)

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()

#     #################################################################

#     # executor = rclpy.executors.MultiThreadedExecutor()
#     # executor.add_node(minimal_publisher)
#     # thread = threading.Thread(target=executor.spin, daemon=True)
#     # thread.start()
#     # minimal_publisher._plt()


# if __name__ == '__main__':
#     main()