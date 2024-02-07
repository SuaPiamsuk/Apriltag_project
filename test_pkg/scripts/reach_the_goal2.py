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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from geometry_msgs.msg import Twist

import math
from math import atan2
from math import sin, cos

import numpy as np
import numpy.matlib as npm

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class MinimalPublisher2(Node):

    def __init__(self):
        super().__init__('minimal_publisher2')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
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
        
        self.docking_state = 'init'

        self.static_broadcaster = StaticTransformBroadcaster(self)
    
    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(
            float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)
    
    def quaternion_translation_to_homogeneous_matrix(self, q, translation):
        ## Example usage:
        # q = [0.7071, 0.0, 0.0, 0.7071]  # Example quaternion
        # translation = [1.0, 2.0, 3.0]  # Example translation vector
        # homogeneous_matrix = quaternion_translation_to_homogeneous_matrix(q, translation)
        # print(homogeneous_matrix)

        w, x, y, z = q

        # Normalize the quaternion
        norm = np.linalg.norm(q)
        if norm != 1.0:
            w /= norm
            x /= norm
            y /= norm
            z /= norm

        # Extract components for clarity
        qx, qy, qz, qw = x, y, z, w

        # Build the 3x3 rotation matrix
        R = np.array([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                    [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
                    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]])

        # Create the 4x4 homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation  # Set the translation vector

        return T

    def matrix_to_quaternion_and_translation(self, matrix):

        # Example usage
        # homogeneous_matrix = np.array([[0.866, -0.5, 0.0, 1.0],
        #                             [0.5, 0.866, 0.0, 2.0],
        #                             [0.0, 0.0, 1.0, 3.0],
        #                             [0.0, 0.0, 0.0, 1.0]])

        # quaternion, translation = matrix_to_quaternion_and_translation(homogeneous_matrix)

        # print("Quaternion:", quaternion)
        # print("Translation:", translation)


        # Extract the rotation part (upper-left 3x3 submatrix)
        rotation_matrix = matrix[:3, :3]

        # Calculate the quaternion from the rotation matrix
        quaternion = np.zeros(4)
        tr = np.trace(rotation_matrix)
        
        if tr > 0:
            S = 0.5 / np.sqrt(tr + 1.0)
            quaternion[0] = 0.25 / S
            quaternion[1] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * S
            quaternion[2] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * S
            quaternion[3] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * S
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                S = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                quaternion[0] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
                quaternion[1] = 0.25 * S
                quaternion[2] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
                quaternion[3] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                S = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                quaternion[0] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
                quaternion[1] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
                quaternion[2] = 0.25 * S
                quaternion[3] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
            else:
                S = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                quaternion[0] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
                quaternion[1] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
                quaternion[2] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
                quaternion[3] = 0.25 * S

        # Extract the translation part (fourth column)
        translation = matrix[:3, 3]

        return quaternion, translation
    
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
            A = np.outer(q,q) + A

        # scale
        A = (1.0/M)*A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:,0].A1)
 
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

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q

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
        if self.apriltagcount == 3:
            tf_avg = TransformStamped()
            tf_avg.header.stamp = self.get_clock().now().to_msg()
            tf_avg.header.frame_id = 'base_footprint'
            tf_avg.child_frame_id = 'docking_station'
            tf_avg.transform.translation.x = (t.transform.translation.x + t2.transform.translation.x + t3.transform.translation.x)/3.0
            tf_avg.transform.translation.y = (t.transform.translation.y + t2.transform.translation.y + t3.transform.translation.y)/3.0
            tf_avg.transform.translation.z = (t.transform.translation.z + t2.transform.translation.z + t3.transform.translation.z)/3.0

            # สร้างเมทริกซ์ Q จาก quaternions
            Q = np.array([
            [t.transform.rotation.x,  t.transform.rotation.y,  t.transform.rotation.z,  t.transform.rotation.w],  # Quaternion 1
            [t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w],  # Quaternion 2
            [t3.transform.rotation.x, t3.transform.rotation.y, t3.transform.rotation.z, t3.transform.rotation.w]   # Quaternion 3
            ])

            # หาค่า quaternion เฉลี่ย
            avg_quaternion = self.averageQuaternions(Q)

            tf_avg.transform.rotation.x = avg_quaternion[0]
            tf_avg.transform.rotation.y = avg_quaternion[1]
            tf_avg.transform.rotation.z = avg_quaternion[2]
            tf_avg.transform.rotation.w = avg_quaternion[3]




            # tf_avg.transform.rotation.x = (t.transform.rotation.x + t2.transform.rotation.x + t3.transform.rotation.x)/3.0
            # tf_avg.transform.rotation.y = (t.transform.rotation.y + t2.transform.rotation.y + t3.transform.rotation.y)/3.0
            # tf_avg.transform.rotation.z = (t.transform.rotation.z + t2.transform.rotation.z + t3.transform.rotation.z)/3.0
            # tf_avg.transform.rotation.w = (t.transform.rotation.w + t2.transform.rotation.w + t3.transform.rotation.w)/3.0

            self.tf_broadcaster.sendTransform(tf_avg)
        
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

            
        try:

            # pre docking distance
            pre_docking_distance = 80.0# percents
            x = t.transform.translation.x*(pre_docking_distance/100.0)
            x = round(x,4)

            # y = t.transform.translation.y*(pre_docking_distance/100.0)
            y = t.transform.translation.y
            y = round(y,4)
            # print(x)
            # print(y)
            # print(t.transform.translation)

            ##############################

            # # roll, pitch, yaw_ = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            # roll, pitch, yaw_ = self.euler_from_quaternion(tf_avg.transform.rotation.x, tf_avg.transform.rotation.y, tf_avg.transform.rotation.z, tf_avg.transform.rotation.w)

        

            # # self.transform_mtx = np.array([[cos(yaw_), -sin(yaw_),   0,     t.transform.translation.x], # นำ matrix มา dot กันเพื่อหาทิศทางปัจจุบัน
            # #                                [sin(yaw_),  cos(yaw_),   0,     t.transform.translation.y], # โดยหมุนแกนในแนวแกน Z
            # #                                [0,             0,        1,     t.transform.translation.z],
            # #                                [0,             0,        0,                          1]])

            # self.transform_mtx = np.array([[cos(yaw_), -sin(yaw_),   0,     tf_avg.transform.translation.x], # นำ matrix มา dot กันเพื่อหาทิศทางปัจจุบัน
            #                             [sin(yaw_),  cos(yaw_),   0,     tf_avg.transform.translation.y], # โดยหมุนแกนในแนวแกน Z
            #                             [0,             0,        1,     tf_avg.transform.translation.z],
            #                             [0,             0,        0,                          1]])
            
            

            # # self.transform_mtx = np.dot(self.transform_mtx,np.array([[1,             0,            0, t.transform.translation.x*(pre_docking_distance/100.0)], # นำ matrix มา dot กันเพื่อหาทิศทางปัจจุบัน
            # #                                                          [0,             1,            0, 0], # โดยหมุนในแนวแกน Z
            # #                                                          [0,             0,            1, 0],
            # #                                                          [0,             0,            0, 1]]))

            # self.transform_mtx = np.dot(self.transform_mtx,np.array([[1,             0,            0, tf_avg.transform.translation.x*(pre_docking_distance/100.0)], # นำ matrix มา dot กันเพื่อหาทิศทางปัจจุบัน
            #                                                         [0,             1,            0, 0], # โดยหมุนในแนวแกน Z
            #                                                         [0,             0,            1, 0],
            #                                                         [0,             0,            0, 1]]))
            

            # angle = np.arctan2(self.transform_mtx[1][0],self.transform_mtx[1][1]) # คำนวณหา heading

            # q = self.quaternion_from_euler(0, 0, angle)

            ##############################

            #angle to quarternion

            # angle = angle*180.0/math.pi
            
            # yaw_ = yaw_*180.0/math.pi

            # # self.get_logger().info('"%s"' % yaw_)
            # self.get_logger().info('"%s"' % angle)
        except:
            pass

        
    
        try:
            ########################
            # q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            # translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z] #translation vector
            # homogeneous_matrix = self.quaternion_translation_to_homogeneous_matrix(q, translation)

            # homogeneous_matrix = np.dot(homogeneous_matrix,np.array([[1.0, 0.0, 0.0, 0.5],
            #                                                         [0.0, 1.0, 0.0, 0.0],
            #                                                         [0.0, 0.0, 1.0, 0.0],
            #                                                         [0.0, 0.0, 0.0, 1.0]]))

            # # homogeneous_matrix to tf_broadcaster
            # quaternion, translation = self.matrix_to_quaternion_and_translation(homogeneous_matrix)
            ########################
            # roll, pitch, yaw_ = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            # print(yaw_*180.0/math.pi)
            ########################

            tf_dock2base_footprint = self.tf_buffer.lookup_transform(
                'docking_station',
                'base_footprint',
                rclpy.time.Time())



            tf_docking = TransformStamped()
            tf_docking.header.stamp = self.get_clock().now().to_msg()
            tf_docking.header.frame_id = 'docking_station'
            # tf_docking.header.frame_id = 'camera_rgb_optical_frame'
            tf_docking.child_frame_id = 'docking'
            tf_docking.transform.translation.x = tf_dock2base_footprint.transform.translation.x*(pre_docking_distance/100.0)
            tf_docking.transform.translation.y = 0.0
            tf_docking.transform.translation.z = 0.0
            # tf_docking.transform.translation.z = tf_dock2base_footprint.transform.translation.z
            # tf_docking.transform.rotation.x = tf_dock2base_footprint.transform.rotation.x
            # tf_docking.transform.rotation.y = tf_dock2base_footprint.transform.rotation.y
            # tf_docking.transform.rotation.z = tf_dock2base_footprint.transform.rotation.z
            # tf_docking.transform.rotation.w = tf_dock2base_footprint.transform.rotation.w
            tf_docking.transform.rotation.x = 0.0
            tf_docking.transform.rotation.y = 0.0
            tf_docking.transform.rotation.z = 0.0
            tf_docking.transform.rotation.w = 1.0

            # tf_docking.header.frame_id = 'base_footprint'
            # # tf_docking.header.frame_id = 'camera_rgb_optical_frame'
            # tf_docking.child_frame_id = 'docking'
            # tf_docking.transform.translation.x = self.transform_mtx[0][3] 
            # tf_docking.transform.translation.y = self.transform_mtx[1][3]
            # tf_docking.transform.translation.z = self.transform_mtx[2][3]
            # tf_docking.transform.rotation.x = q[0]
            # tf_docking.transform.rotation.y = q[1]
            # tf_docking.transform.rotation.z = q[2]
            # tf_docking.transform.rotation.w = q[3]

            

            # tf_docking.header.frame_id = 'odom'
            # tf_docking.child_frame_id = 'docking'
            # tf_docking.transform.translation.x = tf_odom2basefootprint.transform.translation.x
            # tf_docking.transform.translation.y = tf_odom2basefootprint.transform.translation.y
            # tf_docking.transform.translation.z = 0.0
            # tf_docking.transform.rotation.x = tf_odom2basefootprint.transform.rotation.x 
            # tf_docking.transform.rotation.y = tf_odom2basefootprint.transform.rotation.y
            # tf_docking.transform.rotation.z = tf_odom2basefootprint.transform.rotation.z
            # tf_docking.transform.rotation.w = tf_odom2basefootprint.transform.rotation.w
            
            self.tf_broadcaster.sendTransform(tf_docking)
        except:
                pass

        try:
           

            cmd_vel_msg = Twist()

            if self.docking_state == 'init':

                tf_dock2odom = self.tf_buffer.lookup_transform(
                    'odom',
                    'docking',
                    rclpy.time.Time())

                # Publish static transforms once a
                # self.make_transforms(tf_dock2odom)
                static_transform = TransformStamped()
                static_transform.header.frame_id = "odom"
                static_transform.child_frame_id = "pre_docking"
                static_transform.transform.translation.x = tf_dock2odom.transform.translation.x
                static_transform.transform.translation.y = tf_dock2odom.transform.translation.y
                static_transform.transform.translation.z = tf_dock2odom.transform.translation.z
                static_transform.transform.rotation.x = tf_dock2odom.transform.rotation.x 
                static_transform.transform.rotation.y = tf_dock2odom.transform.rotation.y 
                static_transform.transform.rotation.z = tf_dock2odom.transform.rotation.z 
                static_transform.transform.rotation.w = tf_dock2odom.transform.rotation.w 

                # Broadcast the static transform
                self.static_broadcaster.sendTransform(static_transform)

                self.docking_state = 'rotate'

            elif self.docking_state == 'rotate':
                robot2predocking = self.tf_buffer.lookup_transform(
                    'pre_docking',
                    'base_footprint',
                    rclpy.time.Time())
                roll, pitch, yaw = self.euler_from_quaternion( robot2predocking.transform.rotation.x,  robot2predocking.transform.rotation.y,  robot2predocking.transform.rotation.z,  robot2predocking.transform.rotation.w)
                angle_to_goal = atan2(robot2predocking.transform.translation.y, robot2predocking.transform.translation.x)
                pre_yaw = abs(abs(yaw)-math.pi)

                # print(abs(angle_to_goal-pre_yaw))
                # print(abs(angle_to_goal))
                # print(abs(yaw)-math.pi)
                
                

                msg_ = angle_to_goal-pre_yaw
                self.get_logger().info('error: "%s"' % msg_)
                self.get_logger().info('a_z: "%s"' % cmd_vel_msg.angular.z)
                # print(cmd_vel_msg.angular.z)

                if abs(angle_to_goal-pre_yaw) > 0.002:
                    
                    if angle_to_goal-pre_yaw > 0.004:
                        cmd_vel_msg.linear.x = 0.0
                        cmd_vel_msg.angular.z = 0.3
                        print("hhh")
                    elif angle_to_goal-pre_yaw < -0.002:
                        cmd_vel_msg.linear.x = 0.0
                        cmd_vel_msg.angular.z = -0.3
                    
                    # self.docking_state = 'forward'
                    
                
                
                

                    # cmd_vel_msg.angular.z = 0.0
                # else:
                #     cmd_vel_msg.linear.x = 0.0
                #     # cmd_vel_msg.angular.z = -0.02
                #     cmd_vel_msg.angular.z = 0.0
                #     # print("yes")
                #     self.pre_dock_done = 1

                elif abs(angle_to_goal-pre_yaw) < 0.002:
                    self.docking_state = 'forward'

                # cmd_vel_msg.angular.z = -0.4
                self.cmd_publisher.publish(cmd_vel_msg)

                # self.cmd_publisher.publish(cmd_vel_msg)
                print(cmd_vel_msg.angular.z)
            
            elif self.docking_state == 'forward':
                robot2predocking = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    'pre_docking',
                    rclpy.time.Time())
                
                print(robot2predocking.transform.translation.x)
            
                if abs(robot2predocking.transform.translation.x) > 0.01:
                    
                    cmd_vel_msg.linear.x = 0.06
                
                elif abs(robot2predocking.transform.translation.x) < 0.01:
                    self.docking_state = 'align'

                self.cmd_publisher.publish(cmd_vel_msg)
            
            elif self.docking_state == 'align':
                robot2predocking = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    'pre_docking',
                    rclpy.time.Time())
                
                roll, pitch, yaw = self.euler_from_quaternion( robot2predocking.transform.rotation.x,  robot2predocking.transform.rotation.y,  robot2predocking.transform.rotation.z,  robot2predocking.transform.rotation.w)
                angle_to_goal = atan2(robot2predocking.transform.translation.y, robot2predocking.transform.translation.x)
                pre_yaw = abs(abs(yaw)-math.pi)

                # print(abs(angle_to_goal-pre_yaw))
                # print(abs(angle_to_goal))
                # print(abs(yaw)-math.pi)
                
                

                msg_ = yaw
                self.get_logger().info('error: "%s"' % msg_)
                # self.get_logger().info('a_z: "%s"' % cmd_vel_msg.angular.z)
                # print(cmd_vel_msg.angular.z)

                
                if abs(yaw) < 3.11:    
                    if yaw < 3.11:
                        cmd_vel_msg.linear.x = 0.0
                        cmd_vel_msg.angular.z = -0.25
                        print("hhh")
                    elif yaw > -3.11:
                        cmd_vel_msg.linear.x = 0.0
                        cmd_vel_msg.angular.z = 0.25

                elif yaw > 3.11 or yaw < -3.11:
                    cmd_vel_msg.angular.z = 0.0
                    print("exit")
                    self.docking_state = 'exit'

                # # cmd_vel_msg.angular.z = -0.4
                self.cmd_publisher.publish(cmd_vel_msg)

                

            self.get_logger().info('status: "%s"' % self.docking_state)
            
        except:
            pass

        

        try:
            tf_odom2basefootprint = self.tf_buffer.lookup_transform(
                'odom',
                'docking',
                rclpy.time.Time())
            
            # print(tf_odom2basefootprint.transform.translation.x)
            # self.get_logger().info('"%s"' % tf_odom2basefootprint.transform.translation.x)
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



##########################################################################################
        # try:
        #     robot2predocking = self.tf_buffer.lookup_transform(
        #         'docking',
        #         'base_footprint',
        #         rclpy.time.Time())
        # except:
        #     pass

        # #euler from quarternion
        # try:
        #     roll, pitch, yaw = self.euler_from_quaternion( robot2predocking.transform.rotation.x,  robot2predocking.transform.rotation.y,  robot2predocking.transform.rotation.z,  robot2predocking.transform.rotation.w)
        #     angle_to_goal = atan2(t.transform.translation.y,x)
            
        #     pre_yaw = abs(abs(yaw)-math.pi)

        #     # print(abs(angle_to_goal-pre_yaw))
        #     # print(abs(angle_to_goal))
        #     # print(abs(yaw)-math.pi)
            
        #     cmd_vel_msg = Twist()



        #     # if abs(angle_to_goal-pre_yaw) > 0.002 and self.pre_dock_done == 0:
        #     #     cmd_vel_msg.linear.x = 0.0
        #     #     cmd_vel_msg.angular.z = -0.02
        #     #     cmd_vel_msg.angular.z = 0.0
        #     # else:
        #     #     cmd_vel_msg.linear.x = 0.0
        #     #     # cmd_vel_msg.angular.z = -0.02
        #     #     cmd_vel_msg.angular.z = 0.0
        #     #     # print("yes")
        #     #     self.pre_dock_done = 1

            
        #     # self.cmd_publisher.publish(cmd_vel_msg)


        # except:
        #     pass
        

        
             





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher2()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from tf2_ros import StaticTransformBroadcaster
# from geometry_msgs.msg import TransformStamped

# class StaticTransformNode(Node):
#     def __init__(self):
#         super().__init__('static_transform_node')
#         self.static_broadcaster = StaticTransformBroadcaster(self)
#         self.broadcast_static_transform()

#     def broadcast_static_transform(self):
#         static_transform = TransformStamped()
#         static_transform.header.frame_id = "odom"
#         static_transform.child_frame_id = "child_frame"
#         static_transform.transform.translation.x = 1.0  # Adjust these values as needed
#         static_transform.transform.translation.y = 1.0
#         static_transform.transform.translation.z = 0.0
#         static_transform.transform.rotation.x = 0.0
#         static_transform.transform.rotation.y = 0.0
#         static_transform.transform.rotation.z = 0.0
#         static_transform.transform.rotation.w = 1.0

#         # Broadcast the static transform
#         self.static_broadcaster.sendTransform(static_transform)

#         print("hhh")

# def main(args=None):
#     rclpy.init(args=args)
#     static_transform_node = StaticTransformNode()
#     rclpy.spin(static_transform_node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
