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

import os
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import tempfile

def generate_launch_description():
    pkg_rosbridge_server = get_package_share_directory('rosbridge_server')
    pkg_isaac_ros_ess = get_package_share_directory('isaac_ros_ess')
    pkg_realsense = get_package_share_directory('realsense2_camera')
    """Launch file which brings up visual odometry node configured for RealSense."""
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': 2, #https://github.com/IntelRealSense/realsense-ros/issues/817
                'infra_fps': 90.0,
                'unite_imu_method': 'linear_interpolation' # copy | linear_interpolation
        }],
        )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_realsense, '/launch/rs_launch.py'])
    )

    engine_file_path = "/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/ess.engine"

    isaac_ros_dnn_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_isaac_ros_ess, '/isaac_ros_ess.launch.py']),
        launch_arguments={'engine_file_path': engine_file_path}.items()
    )

    disparity_node = ComposableNode(
        name='disparity',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_disparity::ESSDisparityNode',
        parameters=[{'engine_file_path': engine_file_path}],
        remappings=[
            ('left/camera_info', 'infra1/camera_info'),
            ('left/image_rect', 'infra1/image_rect_raw'),
            ('right/camera_info', 'infra2/camera_info'),
            ('right/image_rect', 'infra2/image_rect_raw')
        ]
    )

    container = ComposableNodeContainer(
        name='disparity_container',
        namespace='disparity',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[disparity_node],
        output='screen'
    )

    #rosbridge_server_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        [pkg_rosbridge_server, '/launch/rosbridge_websocket_launch.xml'])
    #)

#        remappings=[
#            ('left/camera_info', 'infra1/camera_info'),
#            ('left/image_rect', 'infra1/image_rect_raw'),
#            ('right/camera_info', 'infra2/camera_info'),
#            ('right/image_rect', 'infra2/image_rect_raw')
#        ]

    return launch.LaunchDescription([realsense_launch, container])