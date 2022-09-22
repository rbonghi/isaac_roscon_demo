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


class ESSVisualizer(Node):

    def __init__(self):
        super().__init__('ess_visualizer')
        self.encoding = 'rgb8'
        # help='Save output or display it.'
        self.declare_parameter("save_image", False)
        self.save_image = bool(self.get_parameter("save_image").value)
        # help='Absolute path to your rosbag.'
        self.declare_parameter("result_path", "/workspaces/isaac_ros-dev/src/output.png")
        self.result_path = self.get_parameter("result_path")
        # help='Use rosbag as inputs or raw image and camera info files as inputs.'
        self.declare_parameter("raw_inputs", False)
        self.raw_inputs = bool(self.get_parameter("raw_inputs").value)

        # help='Absolute path to your rosbag.'
        self.declare_parameter("rosbag_path", "/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/rosbags/ess_rosbag")
        self.rosbag_path = self.get_parameter("rosbag_path")
        # help='Absolute path your left image.'
        self.declare_parameter("left_image_path", "/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/examples/left.png")
        self.left_image_path = self.get_parameter("left_image_path")
        # help='Absolute path your right image.'
        self.declare_parameter("right_image_path", "/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/examples/right.png")
        self.right_image_path = self.get_parameter("right_image_path")
        # help='Absolute path your camera info json file.'
        self.declare_parameter("camera_info_path", "/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/examples/camera.json")
        self.camera_info_path = self.get_parameter("camera_info_path")

        self._bridge = cv_bridge.CvBridge()

        self._disp_sub = self.create_subscription(
            DisparityImage, '/isaac_ros/disparity', self.ess_callback, 10)
        # Output disparity
        self._disparity_view_pub = self.create_publisher(
            Image, 'disparity_view', 10)
        # if self.raw_inputs:
        #     self._prepare_raw_inputs()
        # else:
        #     self._prepare_rosbag_inputs()
        self.get_logger().info(f"ESS visualizer started")

    def _prepare_rosbag_inputs(self):
        subprocess.Popen('ros2 bag play -l ' + self.rosbag_path, shell=True)

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

        left_img = cv2.imread(self.left_image_path)
        right_img = cv2.imread(self.right_image_path)
        self.left_msg = self._bridge.cv2_to_imgmsg(np.array(left_img), self.encoding)
        self.right_msg = self._bridge.cv2_to_imgmsg(np.array(right_img), self.encoding)

        self.camera_info = JSONConversion.load_camera_info_from_json(self.camera_info_path)

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

        self.disparity_msg = self._bridge.cv2_to_imgmsg(np.array(color_map), self.encoding)
        self._disparity_view_pub.publish(self.disparity_msg)
        #if self.save_image:
        #    cv2.imwrite(self.result_path, color_map)
        #else:
        #    cv2.imshow('ess_output', color_map)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    #args = get_args()
    # Start node
    ess_visualizer = ESSVisualizer()
    try:
        rclpy.spin(ess_visualizer)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    ess_visualizer.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF