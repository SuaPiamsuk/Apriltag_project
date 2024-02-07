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

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
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
    
    def listener_callback(self, msg):
        print(len(msg.detections))
        self.apriltagcount = len(msg.detections)

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

        if self.apriltagcount == 3:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = 'base_footprint'
            tf.child_frame_id = 'docking'
            tf.transform.translation.x = (t.transform.translation.x + t2.transform.translation.x + t3.transform.translation.x)/3.0
            tf.transform.translation.y = (t.transform.translation.y + t2.transform.translation.y + t3.transform.translation.y)/3.0
            tf.transform.translation.z = (t.transform.translation.z + t2.transform.translation.z + t3.transform.translation.z)/3.0
            tf.transform.rotation.x = (t.transform.rotation.x + t2.transform.rotation.x + t3.transform.rotation.x)/3.0
            tf.transform.rotation.y = (t.transform.rotation.y + t2.transform.rotation.y + t3.transform.rotation.y)/3.0
            tf.transform.rotation.z = (t.transform.rotation.z + t2.transform.rotation.z + t3.transform.rotation.z)/3.0
            tf.transform.rotation.w = (t.transform.rotation.w + t2.transform.rotation.w + t3.transform.rotation.w)/3.0

            self.tf_broadcaster.sendTransform(tf)
        
        else:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = 'base_footprint'
            tf.child_frame_id = 'docking'
            # tf.transform.translation.x = []
            # tf.transform.translation.y = []
            # tf.transform.translation.z = []
            # tf.transform.rotation.x = []
            # tf.transform.rotation.y = []
            # tf.transform.rotation.z = []
            # tf.transform.rotation.w = []

            self.tf_broadcaster.sendTransform(tf)

        # print(t == [])
        # print(t2 == [])
        # print(t3 == [])




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
