#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#add frame
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from geometry_msgs.msg import Twist

import math
from math import atan2

import numpy
import numpy.matlib as npm

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'apriltag_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.apriltagcount = 0

        self.pre_dock_done = 0
    
    def listener_callback(self, msg):
        # print(len(msg.detections))
        self.apriltagcount = len(msg.detections)
    
    # Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
    # The quaternions are arranged as (w,x,y,z), with w being the scalar
    # The result will be the average quaternion of the input. Note that the signs
    # of the output quaternion can be reversed, since q and -q describe the same orientation
    def averageQuaternions(self, Q):
        # Number of quaternions to average
        M = Q.shape[0]
        A = npm.zeros(shape=(4,4))

        for i in range(0,M):
            q = Q[i,:]
            # multiply q with its transposed version q' and add A
            A = numpy.outer(q,q) + A

        # scale
        A = (1.0/M)*A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = numpy.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return numpy.real(eigenVectors[:,0].A1)
 
    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        from_frame_rel = 'dock_frame1'
        to_frame_rel = 'base_footprint'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            # print(t.transform.translation)
            
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            t=[]
            self.get_logger().info(
                f'Could not transform')
            return
        
        try:
            t2 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                'dock_frame2',
                rclpy.time.Time())
        except:
            t2=[]

        try:
            t3 = self.tf_buffer.lookup_transform(
                to_frame_rel,
                'dock_frame3',
                rclpy.time.Time())
        except:
            t3=[]

        # self.get_logger().info(f'aaa {t}')


        #########################################################################################
        # if self.apriltagcount == 3:
        #     tf = TransformStamped()
        #     tf.header.stamp = self.get_clock().now().to_msg()
        #     tf.header.frame_id = 'base_footprint'
        #     tf.child_frame_id = 'docking'
        #     tf.transform.translation.x = (t.transform.translation.x + t2.transform.translation.x + t3.transform.translation.x)/3.0
        #     tf.transform.translation.y = (t.transform.translation.y + t2.transform.translation.y + t3.transform.translation.y)/3.0
        #     tf.transform.translation.z = (t.transform.translation.z + t2.transform.translation.z + t3.transform.translation.z)/3.0

        #     # สร้างเมทริกซ์ Q จาก quaternions
        #     Q = numpy.array([
        #     [t.transform.rotation.x,  t.transform.rotation.y,  t.transform.rotation.z,  t.transform.rotation.w],  # Quaternion 1
        #     [t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w],  # Quaternion 2
        #     [t3.transform.rotation.x, t3.transform.rotation.y, t3.transform.rotation.z, t3.transform.rotation.w]   # Quaternion 3
        #     ])

        #     # หาค่า quaternion เฉลี่ย
        #     avg_quaternion = self.averageQuaternions(Q)

        #     tf.transform.rotation.x = avg_quaternion[0]
        #     tf.transform.rotation.y = avg_quaternion[1]
        #     tf.transform.rotation.z = avg_quaternion[2]
        #     tf.transform.rotation.w = avg_quaternion[3]




        #     # tf.transform.rotation.x = (t.transform.rotation.x + t2.transform.rotation.x + t3.transform.rotation.x)/3.0
        #     # tf.transform.rotation.y = (t.transform.rotation.y + t2.transform.rotation.y + t3.transform.rotation.y)/3.0
        #     # tf.transform.rotation.z = (t.transform.rotation.z + t2.transform.rotation.z + t3.transform.rotation.z)/3.0
        #     # tf.transform.rotation.w = (t.transform.rotation.w + t2.transform.rotation.w + t3.transform.rotation.w)/3.0

        #     self.tf_broadcaster.sendTransform(tf)
        
        # else:
        #     tf = TransformStamped()
        #     tf.header.stamp = self.get_clock().now().to_msg()
        #     tf.header.frame_id = 'base_footprint'
        #     tf.child_frame_id = 'docking'
        #     # tf.transform.translation.x = []
        #     # tf.transform.translation.y = []
        #     # tf.transform.translation.z = []
        #     # tf.transform.rotation.x = []
        #     # tf.transform.rotation.y = []
        #     # tf.transform.rotation.z = []
        #     # tf.transform.rotation.w = []

        #     self.tf_broadcaster.sendTransform(tf)
        
        #########################################################################################

        # print(t == [])
        # print(t2 == [])
        # print(t3 == [])

        # pre docking distance
        pre_docking_distance = 50.0# percents
        x = t.transform.translation.x*(pre_docking_distance/100.0)
        x = round(x,4)
        print(t.transform.translation.x)
    
        try:
            tf_docking = TransformStamped()
            tf_docking.header.stamp = self.get_clock().now().to_msg()
            tf_docking.header.frame_id = 'base_footprint'
            tf_docking.child_frame_id = 'docking'
            tf_docking.transform.translation.x = x
            tf_docking.transform.translation.y = t.transform.translation.y
            tf_docking.transform.translation.z = 0.0
            tf_docking.transform.rotation.x = t.transform.rotation.x
            tf_docking.transform.rotation.y = t.transform.rotation.y
            tf_docking.transform.rotation.z = t.transform.rotation.z
            tf_docking.transform.rotation.w = t.transform.rotation.w
            # tf.transform.rotation.x = 0.0
            # tf.transform.rotation.y = 0.0
            # tf.transform.rotation.z = 0.0
            # tf.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(tf_docking)
        
        except:
            pass
        # if self.apriltagcount == 3:
        #     tf_docking = TransformStamped()
        #     tf_docking.header.stamp = self.get_clock().now().to_msg()
        #     tf_docking.header.frame_id = 'base_footprint'
        #     tf_docking.child_frame_id = 'dockinggg'
        #     tf_docking.transform.translation.x = x
        #     tf_docking.transform.translation.y = t.transform.translation.y
        #     tf_docking.transform.translation.z = 0.0
        #     tf_docking.transform.rotation.x = t.transform.rotation.x
        #     tf_docking.transform.rotation.y = t.transform.rotation.y
        #     tf_docking.transform.rotation.z = t.transform.rotation.z
        #     tf_docking.transform.rotation.w = t.transform.rotation.w

        #     self.tf_broadcaster.sendTransform(tf_docking)

        try:
            robot2predocking = self.tf_buffer.lookup_transform(
                'docking',
                'base_footprint',
                rclpy.time.Time())
        except:
            pass

        #euler from quarternion
        try:
            roll, pitch, yaw = self.euler_from_quaternion( robot2predocking.transform.rotation.x,  robot2predocking.transform.rotation.y,  robot2predocking.transform.rotation.z,  robot2predocking.transform.rotation.w)
            angle_to_goal = atan2(t.transform.translation.y,x)
            
            pre_yaw = abs(abs(yaw)-math.pi)

            # print(abs(angle_to_goal-pre_yaw))
            # print(abs(angle_to_goal))
            # print(abs(yaw)-math.pi)
            
            cmd_vel_msg = Twist()



            if abs(angle_to_goal-pre_yaw) > 0.002 and self.pre_dock_done == 0:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = -0.02
                cmd_vel_msg.angular.z = 0.0
            else:
                cmd_vel_msg.linear.x = 0.0
                # cmd_vel_msg.angular.z = -0.02
                cmd_vel_msg.angular.z = 0.0
                print("yes")
                self.pre_dock_done = 1

            
            self.cmd_publisher.publish(cmd_vel_msg)
        except:
            pass
        

        
             





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
