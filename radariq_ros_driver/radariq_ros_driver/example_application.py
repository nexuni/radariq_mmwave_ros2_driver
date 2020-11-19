#!/usr/bin/env python

# The MIT License
#
# Copyright (c) 2020 RadarIQ Limited https://radariq.io
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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Example application for listening to the radariq ROS topic
"""

import rclpy
from rclpy.node import Node
from radariq_interfaces.msg import RadarIQObject, RadarIQObjects
from sensor_msgs.msg import PointCloud2


class RadarIQExampleApplication(Node):

    def __init__(self):
        super().__init__('RadarIQExampleApplication')
        self.subscription = self.create_subscription(PointCloud2, 'radariq', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    example_application = RadarIQExampleApplication()
    rclpy.spin(example_application)
    example_application.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()