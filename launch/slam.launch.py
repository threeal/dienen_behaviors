# Copyright (c) 2021 Alfi Maulana
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{
        'frame_id': 'base_footprint',
        'use_sim_time': True,
        'subscribe_depth': True}]

    return LaunchDescription([
        Node(
            package='dienen_behaviors',
            node_executable='odometry_bridge',
            output='screen'
        ),
        Node(
            package='rtabmap_ros',
            node_executable='rtabmap',
            output='screen',
            arguments=['-d'],
            parameters=parameters
        ),
        Node(
            package='rtabmap_ros',
            node_executable='rtabmapviz',
            output='screen',
            parameters=parameters
        ),
    ])
