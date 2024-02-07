#!/usr/bin/env python3

### demo docking with right position of pre-docking 


import os
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#add frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseArray, Pose, Twist

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import math

import numpy as np
# import numpy.matlib as npm

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

def euler_from_quaternion(x, y, z, w):
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

class MinimalPublisher2(Node):

    def __init__(self):
        super().__init__('minimal_publisher2')

        # read config file
        # file_path = os.path.join(os.path.dirname(__file__), "../config/docking.yaml")

        file_path = os.path.join(
        get_package_share_directory('test_pkg'),
        'config',
        'docking_tesr.yaml'
        )

        print("OK")

        with open(file_path, 'r') as file:
            self.config_data = yaml.safe_load(file)

        self.cmd_vel_topic = self.config_data['topic']['cmd_vel']
        self.odom_topic = self.config_data['topic']['odom']

        self.odom_frame = self.config_data['frame']['odom']
        self.base_footprint = self.config_data['frame']['base_footprint']

        self.apriltag_left = self.config_data['apriltag']['left']
        self.apriltag_middle = self.config_data['apriltag']['middle']
        self.apriltag_right = self.config_data['apriltag']['right']
        # print(self.apriltag_left)
        # print(self.apriltag_left['tag_id'])

        # Declare angle metrics in radians
        self.heading_tolerance_ = self.config_data['control']['heading_tolerance']
        self.yaw_goal_tolerance = self.config_data['control']['yaw_goal_tolerance']

        # Declare linear and angular velocities
        self.linear_velocity_max = self.config_data['control']['linear_velocity_max']  # meters per second
        self.angular_velocity_max = self.config_data['control']['angular_velocity_max'] # radians per second

        # Declare distance metrics in meters
        self.distance_goal_tolerance_first_predocking_ = self.config_data['control']['distance_goal_tolerance_first_predocking']
        self.distance_goal_tolerance_next_predocking_ = self.config_data['control']['distance_goal_tolerance_next_predocking']

        self.palabola_alpha = self.config_data['control']['palabola_params']['alpha']

        self.delay_before_start = self.config_data['docking']['delay_before_start']
        self.first_pre_docking_distance = self.config_data['docking']['first_pre_docking_distance']
        self.docking_distance = self.config_data['docking']['docking_distance']
        self.docking_direction = self.config_data['docking']['docking_direction']
    
      

        # Publisher
        self.cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # self.publisherpoints = self.create_publisher(PoseArray, 'points', 10)
        # self.publisherpoint2 = self.create_publisher(Point, 'point2', 10)
        # self.publisherpoint3 = self.create_publisher(Point, 'point3', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'apriltag_detections',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.odom_sub  # prevent unused variable warning

        # Timer
        self.timer_period = 0.08  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.time_ = 0.0
        # self.duration_time = 
        # self.i = 0

        #TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # dynamic
        self.tf_broadcaster = TransformBroadcaster(self) # dynamic
        self.static_broadcaster = StaticTransformBroadcaster(self) #static
        self.tf_odom_to_robot = None
        self.tf_tag25_to_robot = None

        # Kalman filter params 1
        self.K = 0
        self.x = 0
        self.P = 0
        self.P_pre = 0
        self.R = 0.00000015523403536447
        self.C = 1
        self.Q = 0.00009

        # Kalman filter params 2
        self.K1 = 0
        self.x1 = 0
        self.P1 = 0
        self.P_pre1 = 0
        self.R1 = 0.00008339191740914060
        self.C1 = 1
        self.Q1 = 0.00009

        # Initial values
        self.docking_state = 'init'
        self.time_ = 0.0
        self.current_direction = 0.0
        # self.docking_direction = 'forward' # "forward" or "backward"
        self.first_point = True
        self.distance_goal_tolerance = None
        self.reached_distance_goal = False
        self.desired_x = None
        self.desired_y = None
        self.desired_theta = None
        self.current_yaw = None

        self.apriltag_msg = None

        self.kp = 3.0 #2.0
        self.kp_linear = 2.9 #1.99 // 2.2 //2.9

        
    
    def listener_callback(self, msg):
        self.apriltag_msg = msg
    
    def odom_callback(self, msg):
        self.odom_current = PoseStamped()
        self.odom_current.pose = msg.pose.pose

        _, _, current_yaw = euler_from_quaternion(self.odom_current.pose.orientation.x, self.odom_current.pose.orientation.y, self.odom_current.pose.orientation.z, self.odom_current.pose.orientation.w)
        self.current_yaw = round(current_yaw,3)

        self.odom_current.pose.position.x = round(self.odom_current.pose.position.x,3)
        self.odom_current.pose.position.y = round(self.odom_current.pose.position.y,3)

    def kalman(self, y):
            
        self.P_pre = self.P + self.Q
        self.K = (self.P_pre * self.C)/((self.C*self.P_pre*self.C)+self.R)
        self.x = self.x + self.K*(y-self.C*self.x)
        self.P = (1-self.K*self.C)*self.P_pre
        return self.x

    def kalman1(self, y):
            
        self.P_pre1 = self.P1 + self.Q1
        self.K1 = (self.P_pre1 * self.C1)/((self.C1*self.P_pre1*self.C1)+self.R1)
        self.x1 = self.x1 + self.K1*(y-self.C1*self.x1)
        self.P1 = (1-self.K1*self.C1)*self.P_pre1
        return self.x1
    
    def get_distance_to_goal(self):
      """
      Get the distance between the current x,y coordinate and the desired x,y coordinate
      The unit is meters.
      """
      distance_to_goal = math.sqrt(math.pow(self.desired_x - self.odom_current.pose.position.x, 2) + math.pow(
          self.desired_y - self.odom_current.pose.position.y, 2))
      return distance_to_goal
    
    def get_heading_error(self):
        """
        Get the heading error in radians
        """
        delta_x = self.desired_x - self.odom_current.pose.position.x
        delta_y = self.desired_y - self.odom_current.pose.position.y
        desired_heading = math.atan2(delta_y, delta_x) 

        if self.current_direction > 0.0:
            heading_error = desired_heading - self.current_yaw # forward only
        elif self.current_direction < 0.0:
            heading_error = desired_heading - self.current_yaw - math.pi # backward only
        
        # Make sure the heading error falls within -PI to PI range
        if (heading_error > math.pi):
            heading_error = heading_error - (2 * math.pi)
        if (heading_error < -math.pi):
            heading_error = heading_error + (2 * math.pi)
        
        return heading_error
    
    def get_radians_to_goal(self):
        """
        Get the yaw goal angle error in radians
        """    
        yaw_goal_angle_error = self.desired_theta - self.current_yaw

        if self.docking_direction == 'backward':
            if yaw_goal_angle_error < 0:
                yaw_goal_angle_error = yaw_goal_angle_error + math.pi
            elif yaw_goal_angle_error > 0:
                yaw_goal_angle_error = yaw_goal_angle_error - math.pi
      
        return yaw_goal_angle_error
    
   
    def init_starting_point(self):
        try:
            if len(self.apriltag_msg.detections) == 3:
                try: # use try to prevent loss data on current time

                    tf_robot_to_tag_left = self.tf_buffer.lookup_transform(self.base_footprint, self.apriltag_left['frame_name'], rclpy.time.Time()) #left
                    tf_robot_to_tag_middle = self.tf_buffer.lookup_transform(self.base_footprint, self.apriltag_middle['frame_name'], rclpy.time.Time()) #middle
                    tf_robot_to_tag_right = self.tf_buffer.lookup_transform(self.base_footprint, self.apriltag_right['frame_name'], rclpy.time.Time()) #right
                    
                    tfx1 = tf_robot_to_tag_left.transform.translation.x
                    tfy1 = tf_robot_to_tag_left.transform.translation.y
                    tfx2 = tf_robot_to_tag_middle.transform.translation.x
                    tfy2 = tf_robot_to_tag_middle.transform.translation.y
                    tfx3 = tf_robot_to_tag_right.transform.translation.x
                    tfy3 = tf_robot_to_tag_right.transform.translation.y

                    xyxy = np.array([[tfx1,tfy1], [tfx2, tfy2], [tfx3, tfy3]])

                    #find line of best fit
                    coefficients = np.polyfit(xyxy[:, 1], xyxy[:, 0], 1)
                    line_function = np.poly1d(coefficients)

                    y_min = tfy1
                    y_max = tfy3
                    x_min = line_function(y_min)
                    x_max = line_function(y_max)

                    slope = (y_max - y_min) / (x_max - x_min)              

                    slope = -1.0/slope

                    mid_point_x  = (x_max + x_min)/2.0
                    mid_point_y  = (y_max + y_min)/2.0
                
                    xx = 0.0

                    yy = (-1.0 * (mid_point_x-xx) * slope) + mid_point_y

                    yaw_robot_to_predocking = math.atan2((mid_point_y - yy), (mid_point_x - xx)) # calculate yaw of robot to pre_docking


                    tf_docking = TransformStamped()
                    tf_docking.header.stamp = self.get_clock().now().to_msg()
                    tf_docking.header.frame_id = self.odom_frame
                    tf_docking.child_frame_id = 'starting_point'


                    # find tf odom to robot
                    self.tf_odom_to_robot = self.tf_buffer.lookup_transform(self.odom_frame, self.base_footprint, rclpy.time.Time()) #middle

                    tf_docking.transform.translation.x = self.tf_odom_to_robot.transform.translation.x 
                    tf_docking.transform.translation.y = self.tf_odom_to_robot.transform.translation.y 
                    tf_docking.transform.translation.z = self.tf_odom_to_robot.transform.translation.z 
                    
                    _,_,yaw_odom_to_robot = euler_from_quaternion(self.tf_odom_to_robot.transform.rotation.x, self.tf_odom_to_robot.transform.rotation.y, self.tf_odom_to_robot.transform.rotation.z, self.tf_odom_to_robot.transform.rotation.w)
                
                    yaw_odom_to_predocking = yaw_odom_to_robot + yaw_robot_to_predocking # calculate yaw of odom to pre_docking from yaw of robot to pre_docking with yaw of odom to robot
                   
                    q = quaternion_from_euler(0.0, 0.0, yaw_odom_to_predocking) 

                    tf_docking.transform.rotation.x = q[0]
                    tf_docking.transform.rotation.y = q[1]
                    tf_docking.transform.rotation.z = q[2]
                    tf_docking.transform.rotation.w = q[3]
                    self.static_broadcaster.sendTransform(tf_docking)

                except:
                    pass
            else:
                pass
        except:
            pass
    
    def test2(self):
        try:
            # self.get_logger().info('xxx')
            # self.get_logger().info('x: "%s"' % self.apriltag_msg.detections[0].id)
            # self.get_logger().info('x: "%s"' % any(self.apriltag_msg.detections[i].id == 25 for i in range(len(self.apriltag_msg.detections))))

            try:
                tf_tag_25_to_base_footprint = self.tf_buffer.lookup_transform(self.apriltag_middle['frame_name'], self.base_footprint, rclpy.time.Time()) #middle
            except:
                pass
            else:
                self.tf_tag25_to_robot = tf_tag_25_to_base_footprint
            
            
            if len(self.apriltag_msg.detections) == 3 and abs(self.tf_tag25_to_robot.transform.translation.x) >= 0.4:

                try: # use try to prevent loss data on current time
                    self.get_logger().info('a: "%s"' % 'a')

                    tf_robot_to_tag_left = self.tf_buffer.lookup_transform('starting_point', self.apriltag_left['frame_name'], rclpy.time.Time()) #left
                    tf_robot_to_tag_middle = self.tf_buffer.lookup_transform('starting_point', self.apriltag_middle['frame_name'], rclpy.time.Time()) #middle
                    tf_robot_to_tag_right = self.tf_buffer.lookup_transform('starting_point', self.apriltag_right['frame_name'], rclpy.time.Time()) #right
                    self.get_logger().info('b: "%s"' % 'b')
                    
                    tfx1 = tf_robot_to_tag_left.transform.translation.x
                    tfy1 = tf_robot_to_tag_left.transform.translation.y
                    tfx2 = tf_robot_to_tag_middle.transform.translation.x
                    tfy2 = tf_robot_to_tag_middle.transform.translation.y
                    tfx3 = tf_robot_to_tag_right.transform.translation.x
                    tfy3 = tf_robot_to_tag_right.transform.translation.y

                    xyxy = np.array([[tfx1,tfy1], [tfx2, tfy2], [tfx3, tfy3]])

                    #find line of best fit
                    coefficients = np.polyfit(xyxy[:, 1], xyxy[:, 0], 1)
                    line_function = np.poly1d(coefficients)

                    y_min = tfy1
                    y_max = tfy3
                    x_min = line_function(y_min)
                    x_max = line_function(y_max)

                    slope = (y_max - y_min) / (x_max - x_min)

                    slope = -1.0/slope

                    mid_point_x  = (x_max + x_min)/2.0
                    mid_point_y  = (y_max + y_min)/2.0
                    
                    self.tf_starting_point_to_robot = self.tf_buffer.lookup_transform('starting_point', self.base_footprint, rclpy.time.Time()) 
                    self.tf_dist = self.tf_buffer.lookup_transform('starting_point', self.apriltag_middle['frame_name'], rclpy.time.Time()) #middle tag 25


                    # find tf  tag25 to robot 
                    try:
                        tf_tag_25_to_base_footprint = self.tf_buffer.lookup_transform(self.apriltag_middle['frame_name'], self.base_footprint, rclpy.time.Time()) #middle
                    except:
                        pass
                    else:
                        self.tf_tag25_to_robot = tf_tag_25_to_base_footprint



                    # if ( (self.first_pre_docking_distance - self.distance_goal_tolerance_first_predocking_) <= self.tf_tag25_to_robot.transform.translation.z <= (self.first_pre_docking_distance + self.distance_goal_tolerance_first_predocking_)) and abs(self.tf_tag25_to_robot.transform.translation.x)<0.10 :
                    #     self.first_point = False        
                    # if self.first_point == True and (((self.first_pre_docking_distance + self.distance_goal_tolerance_first_predocking_) < self.tf_tag25_to_robot.transform.translation.z < (self.first_pre_docking_distance - self.distance_goal_tolerance_first_predocking_)) or abs(self.tf_tag25_to_robot.transform.translation.x)>0.05 ):
                    #     xx = self.tf_dist.transform.translation.x - self.first_pre_docking_distance
                    
                    if self.first_point == True :

                        if ((self.first_pre_docking_distance + self.distance_goal_tolerance_first_predocking_) < self.tf_tag25_to_robot.transform.translation.x) or (self.tf_tag25_to_robot.transform.translation.x < (self.first_pre_docking_distance - self.distance_goal_tolerance_first_predocking_)): # Ex. 1.0 meters < dist < 0.9 meters
                           
                            xx = self.tf_dist.transform.translation.x - self.first_pre_docking_distance # fixed position of pre-docking
                            
                            
                        elif abs(self.tf_tag25_to_robot.transform.translation.y)>0.05: # error of side > x meters
                            xx = self.tf_dist.transform.translation.x - self.first_pre_docking_distance # fixed position of pre-docking
                          
                        
                        elif (self.first_pre_docking_distance + self.distance_goal_tolerance_first_predocking_) > self.tf_tag25_to_robot.transform.translation.x > (self.first_pre_docking_distance - self.distance_goal_tolerance_first_predocking_): # Ex. 1.0 meters > dist > 0.9 meters: # robot is around pre-docking position and error of side < x meters
                            if abs(self.tf_tag25_to_robot.transform.translation.y)<0.05: # error of side < x meters
                                self.first_point = False
                               
                    
                    else:
                        # palabola equation
                        # scale = (-0.1*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.4 
                        # scale = (-0.0875*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.35
                        scale = (-0.075*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.3 
                        # scale = (-0.0625*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.25 
                        # scale = (-0.05*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.2 
                        # scale = (-0.0375*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.15 
                        # scale = (-0.025*(self.tf_tag25_to_robot.transform.translation.x+2.0)**2) + 0.1
                        # scale = -0.2
                        xx =  self.tf_starting_point_to_robot.transform.translation.x - scale  

                   

                    
                    # try:
                    #     dist = self.tf_buffer.lookup_transform('base_footprint', 'pre_docking', rclpy.time.Time()) #middle tag 25
                    #     self.get_logger().info('x: "%s"' % dist.transform.translation.x)
                    #     self.get_logger().info('y: "%s"' % dist.transform.translation.y)
                    #     self.get_logger().info('self.first_point: "%s"' % self.first_point)
                    # except:
                    #     pass
                    
                    yy = (-1.0 * (mid_point_x-xx) * slope) + mid_point_y
                    
                    yaw = math.atan2((mid_point_y - yy), (mid_point_x - xx))

                    q = quaternion_from_euler(0,0,yaw)
                    
                    tf_docking = TransformStamped()
                    tf_docking.header.stamp = self.get_clock().now().to_msg()
                    tf_docking.header.frame_id = 'starting_point'
                    tf_docking.child_frame_id = 'pre_docking'
                    # tf_docking.transform.translation.x = self.kalman(xx)
                    # tf_docking.transform.translation.y = self.kalman1(yy)
                    tf_docking.transform.translation.x = xx
                    tf_docking.transform.translation.y = yy
                    tf_docking.transform.translation.z = 0.0
                    tf_docking.transform.rotation.x = q[0]
                    tf_docking.transform.rotation.y = q[1]
                    tf_docking.transform.rotation.z = q[2]
                    tf_docking.transform.rotation.w = q[3]
                
                    self.static_broadcaster.sendTransform(tf_docking)                

                except:
                    pass
        

            elif any(self.apriltag_msg.detections[i].id == 25 for i in range(len(self.apriltag_msg.detections))) :
               
                # find tf  tag25 to robot 
                try:
                    tf_tag_25_to_base_footprint = self.tf_buffer.lookup_transform(self.apriltag_middle['frame_name'], self.base_footprint, rclpy.time.Time()) #middle
                except:
                    pass
                else:
                    self.tf_tag25_to_robot = tf_tag_25_to_base_footprint

                if self.first_point == False:
                    try:
                        if abs(self.tf_tag25_to_robot.transform.translation.x) < 0.4: #robot close to station < 0.5 / 0.45 meters // maximum distance to detect 3 tags
                        
                            # q = quaternion_from_euler(-1.57079, 1.57079, 0.0) #rpy = pi/2, pi/2, 0.0
                            q = quaternion_from_euler(0.0, 0.0, 3.141592) #rpy = pi/2, pi/2, 0.0

                            tf_docking = TransformStamped()
                            tf_docking.header.stamp = self.get_clock().now().to_msg() 
                            tf_docking.header.frame_id = self.apriltag_middle['frame_name']
                            tf_docking.child_frame_id = 'pre_docking'
                            # tf_docking.transform.translation.x = 0.0
                            # tf_docking.transform.translation.y = 0.0
                            # tf_docking.transform.translation.z = self.docking_distance
                            tf_docking.transform.translation.x = self.docking_distance
                            # tf_docking.transform.translation.x = 0.38
                            tf_docking.transform.translation.y = 0.0
                            tf_docking.transform.translation.z = 0.0
                            tf_docking.transform.rotation.x = q[0]
                            tf_docking.transform.rotation.y = q[1]
                            tf_docking.transform.rotation.z = q[2]
                            tf_docking.transform.rotation.w = q[3]
                        
                            self.static_broadcaster.sendTransform(tf_docking)

                            
                    except:
                        pass
            else:
                pass

            # # self.get_logger().info('x: "%s"' % any(self.apriltag_msg.detections[i].id[0] == 25 for i in range(len(self.apriltag_msg.detections))))
            # self.get_logger().info('xxx')
        
        except:
            pass
    
    def test3(self):
        cmd_vel_msg = Twist()
        try:
            robot2predocking = self.tf_buffer.lookup_transform(self.odom_frame, 'pre_docking', rclpy.time.Time())

            robot2predocking_2 = self.tf_buffer.lookup_transform(self.base_footprint, 'pre_docking', rclpy.time.Time())

            # self.get_logger().info('test: "%s"' % 'err')

            # cal direction
            if robot2predocking_2.transform.translation.x >= 0.0 :
                self.current_direction = 1.0
            elif robot2predocking_2.transform.translation.x < 0.0 :
                self.current_direction = -1.0
                
            # defined x,y for caliculate theta
            self.desired_x = round(robot2predocking.transform.translation.x,3)
            self.desired_y = round(robot2predocking.transform.translation.y,3)

            _, _, self.desired_theta = euler_from_quaternion(robot2predocking.transform.rotation.x , robot2predocking.transform.rotation.y, robot2predocking.transform.rotation.z, robot2predocking.transform.rotation.w)
            self.desired_theta = round(self.desired_theta,3)

            #cal error (linear and angular)
            distance_to_goal = self.get_distance_to_goal()
            heading_error = self.get_heading_error()
            yaw_goal_error = self.get_radians_to_goal()

            print(self.tf_tag25_to_robot.transform.translation.x)

            #tolerance depend on distance to pre-docking
            if self.first_point == True:
                if (self.first_pre_docking_distance - self.distance_goal_tolerance_first_predocking_) >= abs(self.tf_tag25_to_robot.transform.translation.x) or abs(self.tf_tag25_to_robot.transform.translation.x) >= (self.first_pre_docking_distance + self.distance_goal_tolerance_first_predocking_) :
                    self.distance_goal_tolerance = self.distance_goal_tolerance_first_predocking_ 

                elif abs(self.tf_tag25_to_robot.transform.translation.y) >= self.distance_goal_tolerance_first_predocking_ :
                    self.distance_goal_tolerance = self.distance_goal_tolerance_first_predocking_ 
            else:
                self.distance_goal_tolerance = self.distance_goal_tolerance_next_predocking_
            
            # PID
            if (math.fabs(distance_to_goal) > self.distance_goal_tolerance and self.reached_distance_goal == False ):
                # self.get_logger().info("1")
                
                                    
                # if (math.fabs(heading_error) > 0.2): #and abs(self.tf_tag25_to_robot.transform.translation.z) > 0.5: # > 0.5, 0.6 ถ้าระยะน้อยกว่า x จะไม่ปรับ major heading  
                if (math.fabs(heading_error) > 0.2) and abs(self.tf_tag25_to_robot.transform.translation.x) > 0.5: # > 0.5, 0.6 ถ้าระยะน้อยกว่า x จะไม่ปรับ major heading  
                # if (math.fabs(heading_error) > 0.2) :
                    self.get_logger().info("1.1")
                    self.get_logger().info('1.1: "%s"' % math.fabs(heading_error))
                    cmd_vel_msg.angular.z = self.kp * heading_error

                else: # heading_error = 0.3 and dist err > 0
                    self.get_logger().info("1.2")
                    
                    cmd_vel_msg.linear.x = self.current_direction * self.kp_linear * distance_to_goal
                    cmd_vel_msg.angular.z = self.kp * heading_error
                    
                    # option 1
                    # if (abs(self.desired_y - self.odom_current.pose.position.y) < 0.01) and abs(self.tf_tag25_to_robot.transform.translation.x) < 0.55: # < 0.45, 0.55 # ถ้าระยะน้อยกว่า x จะไม่ปรับ heading
                    #     self.get_logger().info("1.3")
                    #     cmd_vel_msg.angular.z = 0.0 #ไม่ต้องปรับ heading เมือระยะใกล้ถึง station และ error y น้อยกว่า 1 cm

                    # # option 2
                    # if (math.fabs(heading_error) < self.heading_tolerance):
                    #     cmd_vel_msg.angular.z = 0.0

                    #option 3
                    if abs(self.tf_tag25_to_robot.transform.translation.x) < 0.4: # < 0.4, 0.45, 0.55 # ถ้าระยะน้อยกว่า x จะไม่ปรับ heading
                        self.get_logger().info("1.3")
                        cmd_vel_msg.angular.z = 0.0 #ไม่ต้องปรับ heading เมือระยะใกล้ถึง station และ error y น้อยกว่า

                        
                    
            # Orient towards the yaw goal angle
            # elif (math.fabs(yaw_goal_error) > self.yaw_goal_tolerance) and (abs(self.tf_tag25_to_robot.transform.translation.x) > self.docking_distance + self.distance_goal_tolerance_next_predocking_ + 0.1) : # ถ้าระยะมากกว่า x จะปรับ heading ตอนไปถึง goal  
            elif (math.fabs(yaw_goal_error) > self.yaw_goal_tolerance) and (abs(self.tf_tag25_to_robot.transform.translation.x) > 0.45):
                self.get_logger().info("2")
                cmd_vel_msg.angular.z = (self.kp) * yaw_goal_error

                self.reached_distance_goal = True

                # self.get_logger().info("i found it")
            
            # Goal achieved, go to the next goal  
            else:
                # self.get_logger().info("3")
                
                # Go to the next goal
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_publisher.publish(cmd_vel_msg)

                self.reached_distance_goal = False
                self.first_point = False

                # 'Arrived at perpendicular line. Going straight to AprilTag...'
                self.reached_distance_goal = False   

                # self.get_logger().info('c: "%s"' % abs(self.tf_tag25_to_robot.transform.translation.x))

                if abs(self.tf_tag25_to_robot.transform.translation.x) <= self.docking_distance + self.distance_goal_tolerance : # 0.X + tolerance
                    try:
                        report_error = self.tf_buffer.lookup_transform('base_footprint', self.apriltag_middle['frame_name'], rclpy.time.Time()) 
                        _,_,yaw_report_error = euler_from_quaternion(report_error.transform.rotation.x, report_error.transform.rotation.y, report_error.transform.rotation.z, report_error.transform.rotation.w)
                        yaw_report_error = abs(yaw_report_error)  
                        yaw_report_error = round(math.degrees(yaw_report_error),2)
                        # print("report error -> dist_y_error = {} meters / angle_error = {} degrees".format(round(report_error.transform.translation.y,3), yaw_report_error))

                        self.get_logger().info('dist_y_error = "%s"' % round(report_error.transform.translation.y,3))
                        self.get_logger().info('angle_error = "%s"' % yaw_report_error)

                        self.docking_state = 'succeed'
                        # print(self.docking_state)
                        self.get_logger().info("succeed")

                    except:
                        # print("waitting for report...")
                        self.get_logger().info("waitting for report...")
                
                # pass
        except:
            pass  

        if cmd_vel_msg.linear.x > self.linear_velocity_max:
            cmd_vel_msg.linear.x = self.linear_velocity_max
        elif cmd_vel_msg.linear.x < -self.linear_velocity_max:
            cmd_vel_msg.linear.x = -self.linear_velocity_max

        if cmd_vel_msg.angular.z > self.angular_velocity_max:
            cmd_vel_msg.angular.z = self.angular_velocity_max
        elif cmd_vel_msg.angular.z < -self.angular_velocity_max:
            cmd_vel_msg.angular.z = -self.angular_velocity_max
        
        # cmd_vel_msg.linear.x = 0.1
        self.cmd_publisher.publish(cmd_vel_msg)

        # self.get_logger().info('c: "%s"' % abs(self.tf_tag25_to_robot.transform.translation.x))
        # self.get_logger().info('d: "%s"' % cmd_vel_msg.angular.z)

        # print(self.docking_state)


    def timer_callback(self):
        # cmd_vel_msg = Twist()

        if self.docking_state == 'init':
            self.time_ = 0.0
            self.reached_distance_goal = False 
            
            self.docking_state = 'waitting'
        
        elif self.docking_state == 'waitting':
            self.time_ = self.time_ + self.timer_period
            if self.time_ >= 2.0:
                self.init_starting_point()
                try:
                    if len(self.apriltag_msg.detections) == 3:
                        self.time_ = 0.0
                        self.docking_state = 'waitting2'
                except:
                    pass
        elif self.docking_state == 'waitting2':
            self.time_ = self.time_ + self.timer_period
            self.test2()
            if self.time_ >= 2.0: #prevent missing tf
                self.docking_state = 'going to the docking'
        
        elif self.docking_state == 'going to the docking':
            self.test2()
            self.test3()

    #    print("okk")
    # #    self.get_logger().info('x: "%s"' % yaw_odom_to_predocking)
    #    self.get_logger().info('xx:')
    


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