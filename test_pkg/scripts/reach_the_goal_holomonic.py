#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

"""
Subscribe velocity from twist_mux and apply velocity profile.
Use S-curve velocity profile. Calculate acceleration to be trapezodial
then calculate velocity using calculated acceleration and feedback velocity from robot.
"""

class velocity_filter(Node):

    def __init__(self):
        super().__init__('vel_filter')
        self.timer = self.create_timer(1.0, self.update)
        
        self.LIMIT_linear_vel = 0.1
        self.LIMIT_angular_vel = 0.5
        self.LIMIT_linear_acc = 0.5
        self.LIMIT_angular_acc = 0.5

        ###################################
        self.LIMIT_linear_jrk = 3.0
        # self.LIMIT_angular_jrk = rospy.get_param("/config/limit_ang_jrk", 0.5)
        self.current_lin_acc = 0
        # self.current_ang_acc = 0
        self.target_linear_acc = 0.0
        self.target_angular_acc = 0.0
        # self.acc_pub = rospy.Publisher("/acc", Float32, queue_size=10)
        ###################################
        
        
        self.omni = True
        self.target_linear_vel_x = 0.0
        self.target_linear_vel_y = 0.0
        self.target_angular_vel = 0.0
        self.freq = 20.0
        self.rate = self.freq
        # rospy.loginfo("Start velocity filter :: freq {} Hz".format(self.freq))
        self.pub = self.create_publisher(Twist,'cmd_vel' ,50)
        self.sub_postvel = self.create_subscription(Twist
            ,'cmd_vel/test' , self.filtering_callback, 20)

        self.feedback_vel_sub = self.create_subscription(Odometry,'odom', self.feedback_vel_callback,10)
        self.current_vel = Odometry()

        self.twist = Twist()
        self.pub_twist = Twist()
        self.forward_factor = 1.0
        self.backward_factor = 1.0
        self.spin_factor = 1.0

        self.param_cnt = 0
        self.safety_cnt = 0
        self.emergency_break = False

    def feedback_vel_callback(self,data):
        self.current_vel = data

    def filtering_callback(self, data):
        """
        subscribe velocity command from twist_mux
        """
        self.target_linear_vel_x = data.linear.x
        self.target_angular_vel = data.angular.z
        if self.target_linear_vel_x > self.LIMIT_linear_vel:
            self.target_linear_vel_x = self.LIMIT_linear_vel
        if self.target_linear_vel_x < -self.LIMIT_linear_vel:
            self.target_linear_vel_x = -self.LIMIT_linear_vel

        if self.target_angular_vel > self.LIMIT_angular_vel:
            self.target_angular_vel = self.LIMIT_angular_vel
        if self.target_angular_vel < -self.LIMIT_angular_vel:
            self.target_angular_vel = -self.LIMIT_angular_vel

        if self.omni:
            if self.target_linear_vel_y > self.LIMIT_linear_vel:
                self.target_linear_vel_y = self.LIMIT_linear_vel
            if self.target_linear_vel_y < -self.LIMIT_linear_vel:
                self.target_linear_vel_y = -self.LIMIT_linear_vel
            self.target_linear_vel_y = data.linear.y

    def makeSimpleProfile(self, output, input, slop):
        """
        Make simple trapezodial profile graph
        """
        if input > output:
            output = min(input, output + (slop / self.freq))
        elif input < output:
            output = max(input, output - (slop / self.freq))
        else:
            output = input

        return output

    def reset(self):
        """
        reset both linear and angular velocity to 0 
        """
        self.twist = Twist()
        self.pub.publish(self.twist)

    def update(self):
        """
        Step are
        1. Update max velocity and acceleration, get emergency brake and movement factor
        2. check in case abs(command velocity) > abs(limit velocity) 
        3. Calculate curent acceleration
        4. Calculate Velocity
        5. Multiply velocity by movement_factor(from safety sensor)
        6. Check for emergency brake button from joystick
        7. Publish velocity.  
        """

        if self.param_cnt >= int(self.freq):
            
            self.LIMIT_linear_vel = 0.1
            self.LIMIT_angular_vel = 0.5
            self.LIMIT_linear_acc = 0.5
            self.LIMIT_angular_acc = 0.5

            self.param_cnt = 0

        if self.safety_cnt >= 3:  # freq = self.freq/2.0
            self.emergency_break = False
            self.forward_factor = 1.0
            self.backward_factor = 1.0
            self.spin_factor = 1.0
            self.safety_cnt = 0

        ##################################
        d_linear_vel = self.target_linear_vel_x - self.current_vel.twist.twist.linear.x
        if(abs(d_linear_vel) < 0.01 ):
            self.target_linear_acc = 0
        elif(d_linear_vel < 0 ):
            self.target_linear_acc = -self.LIMIT_linear_acc
        elif(d_linear_vel > 0 ):
            self.target_linear_acc = self.LIMIT_linear_acc

        d_angular_vel = self.target_angular_vel - self.current_vel.twist.twist.angular.z
        if(abs(d_angular_vel) < 0.01):
            self.target_angular_acc = 0
        elif(d_angular_vel < 0 ):
            self.target_angular_acc = -self.LIMIT_angular_acc
        elif(d_angular_vel > 0 ):
            self.target_angular_acc = self.LIMIT_angular_acc
            
                
        self.current_lin_acc = self.makeSimpleProfile(
            self.current_lin_acc, self.target_linear_acc, self.LIMIT_linear_jrk )
        # self.current_ang_acc = self.makeSimpleProfile(
        #     self.current_ang_acc, self.target_angular_acc, self.LIMIT_angular_jrk )
        # acc_msg = Float32()
        # acc_msg.data = self.current_lin_acc
        # self.acc_pub.publish(acc_msg)
        # print(self.current_lin_acc)
        ##################################

        

        

        self.twist.linear.x = self.makeSimpleProfile(
            self.twist.linear.x, self.target_linear_vel_x, abs( self.current_lin_acc))
        if(abs(d_linear_vel) < 0.01 and self.target_linear_vel_x==0 ):
            self.twist.linear.x = 0.0
        # self.twist.angular.z = self.makeSimpleProfile(
        #     self.twist.angular.z, self.target_angular_vel, self.LIMIT_angular_acc)

        self.twist.angular.z = self.makeSimpleProfile(
            self.current_vel.twist.twist.angular.z, self.target_angular_vel, self.LIMIT_angular_acc)

        if self.omni:
            self.twist.linear.y = self.makeSimpleProfile(
                self.current_vel.twist.twist.linear.y, self.target_linear_vel_y, self.LIMIT_linear_vel)

        print(self.twist.angular.z)
        print(self.target_angular_vel)
        print(self.current_vel.twist.twist.angular.z)
    # -------------------------------------------------------------------------------------------------#
        # self.pub_twist.linear.x = self.twist.linear.x
        # self.pub_twist.linear.y = self.twist.linear.y
        # self.pub_twist.angular.z = self.twist.angular.z * self.spin_factor
        # if self.twist.linear.x > 0.0:
        #     self.pub_twist.linear.x = self.twist.linear.x * self.forward_factor
        # elif self.twist.linear.x < 0.0:
        #     self.pub_twist.linear.x = self.twist.linear.x * self.backward_factor
        
        if self.twist.linear.x > 0.0:
            self.twist.linear.x *= self.forward_factor
        elif self.twist.linear.x < 0.0:
            self.twist.linear.x *= self.backward_factor
        self.pub_twist.linear.x = self.twist.linear.x
        self.pub_twist.linear.y = self.twist.linear.y
        self.pub_twist.angular.z = self.twist.angular.z * self.spin_factor
        
        if self.emergency_break:
            self.twist = Twist()
            self.pub_twist = Twist()

        # print("-------")
        # print(str(self.target_linear_vel_x)+" "+str(self.target_angular_vel))
        # print(self.twist)
        # print("=======")
        
        self.pub.publish(self.pub_twist)
        self.param_cnt += 1
        self.safety_cnt += 1
        # self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    Vel = velocity_filter()
    rclpy.spin(Vel)
    Vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()