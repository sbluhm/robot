# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam launch')
    parser.add_argument(
        '-n', '--node-name', dest='node_name', type=str, help='name for device', default='usb_cam'
    )

    args, unknown = parser.parse_known_args(sys.argv[4:])

    usb_cam_dir = get_package_share_directory('v4l2_camera')

    # get path to params file
    params_path = os.path.join(bluhmbot, 'config', 'v4l2_camera.yaml')

    node_name = args.node_name

    print(params_path)
    ld.add_action(
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            name=node_name,
            namespace='camera',
            parameters=[
                params_path,
                {
                    '.image_raw.ffmpeg.encoding': 'h264_vaapi',
                    '.image_raw.ffmpeg.profile': 'main',
                    '.image_raw.ffmpeg.preset': 'll',
                    '.image_raw.ffmpeg.gop': 15,
                    '.image_raw.ffmpeg.qmax': 31,
                },
            ],
        )
    )

    return ld

