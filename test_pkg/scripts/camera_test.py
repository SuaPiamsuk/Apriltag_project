#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np
from message_filters import TimeSynchronizer

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.subscription = self.create_subscription(Image, 'fisheye2/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Image, 't265_camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 't265_camera/camera_info', 10)
        self.cv_bridge = CvBridge()

        file_path = '/home/utac01r/ros2_ws/src/test_pkg/config/realsense_T265_calibrated.yaml'
        with open(file_path, 'r') as file:
            self.calibrated_data = yaml.safe_load(file)
        
        # Read parameters from yaml file.
        img_height = self.calibrated_data['image_height']
        img_width = self.calibrated_data['image_width']
        D = self.calibrated_data['distortion_coefficients']['data']
        K = self.calibrated_data['camera_matrix']['data']
        R = self.calibrated_data['rectification_matrix']['data']
        P = self.calibrated_data['projection_matrix']['data']
        
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_msg.header.frame_id = 'camera_fisheye2_optical_frame'
        self.camera_info_msg.height = img_height
        self.camera_info_msg.width = img_width
        self.camera_info_msg.distortion_model = self.calibrated_data['distortion_model']
        self.camera_info_msg.d = D
        self.camera_info_msg.k = K
        self.camera_info_msg.r = R
        self.camera_info_msg.p = P
        self.camera_info_msg.roi.do_rectify = True

    def callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()

        self.camera_info_msg.header.stamp = current_time

        # # Read parameters from yaml file.
        # img_height = self.calibrated_data['image_height']
        # img_width = self.calibrated_data['image_width']
        # D = self.calibrated_data['distortion_coefficients']['data']
        # K = self.calibrated_data['camera_matrix']['data']
        # R = self.calibrated_data['rectification_matrix']['data']
        # P = self.calibrated_data['projection_matrix']['data']
        

        # self.camera_info_msg = CameraInfo()
        # self.camera_info_msg.height = img_height
        # self.camera_info_msg.width = img_width
        # self.camera_info_msg.distortion_model = self.calibrated_data['distortion_model']
        # self.camera_info_msg.d = D
        # self.camera_info_msg.k = K
        # self.camera_info_msg.r = R
        # self.camera_info_msg.p = P



        # Process the image here (for example, convert it to grayscale)
        # img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # processed_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        
        # h = self.calibrated_data['image_height']
        # w = self.calibrated_data['image_width']
        # h = 700
        # w = 760
        # mtx = np.array([self.calibrated_data['camera_matrix']['data']])
        # dist = np.array([self.calibrated_data['distortion_coefficients']['data']])
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # x, y, w, h = roi
        # dst = dst[y : y + h, x : x + w]

        # DIM=(848, 800)




        DIM = (self.calibrated_data['image_width'], self.calibrated_data['image_height'])
        
        K = np.array(self.calibrated_data['camera_matrix']['data'])
        K = np.reshape(K, [3,3])

        # K = np.array([[286.47223115026156, 0.0, 419.56356783213585], [0.0, 286.62796669442713, 395.2738810025333], [0.0, 0.0, 1.0]])
        
        D = np.array(self.calibrated_data['distortion_coefficients']['data'])
        D = D[:4]


        # D = np.array([[-0.007743336635121026], [0.050979370446699375], [-0.0501209168047957], [0.010355519447439171]])

        # print(np.eye(3))
        P = np.array(self.calibrated_data['projection_matrix']['data'])
        P = np.reshape(P, [3,4])

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_32FC1)
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, DIM, cv2.CV_16SC2)
        # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)


        # print(self.calibrated_data['camera_name'])

        # Republish the processed image
        # output_msg = self.cv_bridge.cv2_to_imgmsg(processed_image)
        # output_msg = self.cv_bridge.cv2_to_imgmsg(dst)

        output_msg = self.cv_bridge.cv2_to_imgmsg(undistorted_img, 'mono8')
        output_msg.header.stamp = current_time
        output_msg.header.frame_id = 'camera_fisheye2_optical_frame'

        # self.get_logger().info('cam_info: "%s"' % self.camera_info_msg.header.stamp)
        # self.get_logger().info('cam_img: "%s"' % output_msg.header.stamp)

        self.camera_info_publisher.publish(self.camera_info_msg)
        self.publisher.publish(output_msg)

        # Publish the CameraInfo message
        # self.get_logger().info("Publishing CameraInfo")
        # self.camera_info_publisher.publish(self.camera_info_msg)

        # self.get_logger().info(self.camera_info_msg.header.stamp)

        # self.get_logger().info('cam_info: "%s"' % self.camera_info_msg.header.stamp)
        # self.get_logger().info('cam_img: "%s"' % output_msg.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()