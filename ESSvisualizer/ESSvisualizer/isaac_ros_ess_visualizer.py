# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import argparse
import subprocess

import cv2
import cv_bridge
from isaac_ros_test import JSONConversion
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage


def get_args():
    parser = argparse.ArgumentParser(description='ESS Disparity Node Visualizer')
    parser.add_argument('--save_image', action='store_true', help='Save output or display it.')
    parser.add_argument('--result_path', default='/workspaces/isaac_ros-dev/src/output.png',
                        help='Absolute path to save your result.')
    parser.add_argument('--raw_inputs', action='store_true',
                        help='Use rosbag as inputs or raw image and camera info files as inputs.')
    parser.add_argument('--rosbag_path',
                        default='/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/'
                                'resources/rosbags/ess_rosbag',
                        help='Absolute path to your rosbag.')
    parser.add_argument('--left_image_path',
                        default='/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/'
                                'resources/examples/left.png',
                        help='Absolute path your left image.')
    parser.add_argument('--right_image_path',
                        default='/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/'
                                'resources/examples/right.png',
                        help='Absolute path your right image.')
    parser.add_argument('--camera_info_path',
                        default='/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/'
                                'resources/examples/camera.json',
                        help='Absolute path your camera info json file.')
    args = parser.parse_args()
    return args


class ESSVisualizer(Node):

    def __init__(self, args):
        super().__init__('ess_visualizer')
        self.args = args
        self.encoding = 'rgb8'
        self._bridge = cv_bridge.CvBridge()

        self._disp_sub = self.create_subscription(
            DisparityImage, 'disparity', self.ess_callback, 10)

        # if self.args.raw_inputs:
        #     self._prepare_raw_inputs()
        # else:
        #     self._prepare_rosbag_inputs()

    def _prepare_rosbag_inputs(self):
        subprocess.Popen('ros2 bag play -l ' + self.args.rosbag_path, shell=True)

    def _prepare_raw_inputs(self):
        self._img_left_pub = self.create_publisher(
            Image, 'left/image_rect', 10)
        self._img_right_pub = self.create_publisher(
            Image, 'right/image_rect', 10)
        self._camera_left_pub = self.create_publisher(
            CameraInfo, 'left/camera_info', 10)
        self._camera_right_pub = self.create_publisher(
            CameraInfo, 'right/camera_info', 10)

        self.create_timer(5, self.timer_callback)

        left_img = cv2.imread(self.args.left_image_path)
        right_img = cv2.imread(self.args.right_image_path)
        self.left_msg = self._bridge.cv2_to_imgmsg(np.array(left_img), self.encoding)
        self.right_msg = self._bridge.cv2_to_imgmsg(np.array(right_img), self.encoding)

        self.camera_info = JSONConversion.load_camera_info_from_json(self.args.camera_info_path)

    def timer_callback(self):
        self._img_left_pub.publish(self.left_msg)
        self._img_right_pub.publish(self.right_msg)
        self._camera_left_pub.publish(self.camera_info)
        self._camera_right_pub.publish(self.camera_info)
        self.get_logger().info('Inputs were published.')

    def ess_callback(self, disp_msg):
        self.get_logger().info('Result was received.')
        disp_img = self._bridge.imgmsg_to_cv2(disp_msg.image)
        # Normalize and convert to colormap for visualization
        disp_img = (disp_img - disp_img.min()) / disp_img.max() * 255
        color_map = cv2.applyColorMap(disp_img.astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        if self.args.save_image:
            cv2.imwrite(self.args.result_path, color_map)
        else:
            cv2.imshow('ess_output', color_map)
        cv2.waitKey(1)


def main():
    args = get_args()
    rclpy.init()
    # Start node
    ess_visualizer = ESSVisualizer(args)
    rclpy.spin(ess_visualizer)
    # Destroy the node explicitly
    ess_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF