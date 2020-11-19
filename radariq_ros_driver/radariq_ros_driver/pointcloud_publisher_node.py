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
Publisher node for publishing controlling the RadarIQ module and publishing
to the radariq ROS topic
"""

import signal
import rclpy
import ctypes
import struct
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from radariq import RadarIQ, MODE_POINT_CLOUD

riq = None


class PointCloudPublisher(Node):

    def __init__(self):
        super().__init__('RadarIQPointCloudNode')
        global riq

        serial_port = self.declare_parameter('serial_port').value
        framerate = self.declare_parameter('framerate').value
        distancefilter_min = self.declare_parameter('distancefilter_min').value
        distancefilter_max = self.declare_parameter('distancefilter_max').value
        anglefilter_min = self.declare_parameter('anglefilter_min').value
        anglefilter_max = self.declare_parameter('anglefilter_max').value
        pointdensity = self.declare_parameter('pointdensity').value
        certainty = self.declare_parameter('certainty').value
        topic = self.declare_parameter('topic').value
        frame_id = self.declare_parameter('frame_id').value

        pub = self.create_publisher(PointCloud2, topic, 10)

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
                  ]

        header = Header()
        header.frame_id = frame_id

        try:
            riq = RadarIQ(port=serial_port)
            riq.set_mode(MODE_POINT_CLOUD)
            riq.set_units('m', 'm/s')
            riq.set_frame_rate(framerate)
            riq.set_distance_filter(distancefilter_min, distancefilter_max)
            riq.set_angle_filter(anglefilter_min, anglefilter_max)
            riq.set_point_density(pointdensity)
            riq.set_certainty(certainty)
            riq.start()
            self.get_logger().info('Starting the RadarIQ module')

            for row in riq.get_data():
                if not rclpy.ok():
                    break
                if row is not None:
                    pc2 = self.create_cloud(header, fields, row)
                    pc2.header.stamp = self.get_clock().now().to_msg()
                    pub.publish(pc2)

        except Exception as error:
            self.get_logger().error(error)
        finally:
            del riq
            self.get_logger().info("Stopped RadarIQ module")

    def create_cloud(self, header, fields, points):
        """
        Based off sensor_msgs/point_cloud2.py
        """
        cloud_struct = struct.Struct("<ffff")
        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0
        for p in points:
            pack_into(buff, offset, *p)
            offset += point_step

        return PointCloud2(header=header,
                           height=1,
                           width=len(points),
                           is_dense=False,
                           is_bigendian=False,
                           fields=fields,
                           point_step=cloud_struct.size,
                           row_step=cloud_struct.size * len(points),
                           data=buff.raw)


def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()
    rclpy.spin(pointcloud_publisher)

    # pointcloud_publisher.destroy_node()
    rclpy.shutdown()


def signal_handler(sig, frame):
    global riq
    try:
        riq.close()
    except:
        pass


if __name__ == '__main__':
    main()
