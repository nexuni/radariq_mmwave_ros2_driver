#!/usr/bin/env python


import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from radariq_interfaces.msg import RadarIQObject, RadarIQObjects
from radariq import RadarIQ, MODE_OBJECT_TRACKING, OUTPUT_LIST

riq = None


class ObjectPublisher(Node):

    def __init__(self):
        super().__init__('RadarIQObjectNode')
        global riq

        self.colors = [[0, 0.5, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0.55, 0.27, 0.7],
                       [0, 1, 1], [1, 0, 1], [0.39, 0.58, 0.93], [1, 0.41, 0.71], [1, 0.89, 0.77]]

        serial_port = self.declare_parameter('serial_port').value
        framerate = self.declare_parameter('framerate').value
        distancefilter_min = self.declare_parameter('distancefilter_min').value
        distancefilter_max = self.declare_parameter('distancefilter_max').value
        anglefilter_min = self.declare_parameter('anglefilter_min').value
        anglefilter_max = self.declare_parameter('anglefilter_max').value
        pointdensity = self.declare_parameter('pointdensity').value
        certainty = self.declare_parameter('certainty').value
        marker_topic = self.declare_parameter('marker_topic').value
        object_data_topic = self.declare_parameter('object_data_topic').value
        self.frame_id = self.declare_parameter('frame_id').value

        marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        object_publisher = self.create_publisher(RadarIQObjects, object_data_topic, 10)

        header = Header()
        header.frame_id = self.frame_id

        try:
            riq = RadarIQ(port=serial_port)
            riq.set_mode(MODE_OBJECT_TRACKING)
            riq.set_units('m', 'm/s')
            riq.set_frame_rate(framerate)
            riq.set_distance_filter(distancefilter_min, distancefilter_max)
            riq.set_angle_filter(anglefilter_min, anglefilter_max)
            riq.set_point_density(pointdensity)
            riq.set_certainty(certainty)
            riq.start()
            self.get_logger().info("Starting the RadarIQ module")

            for row in riq.get_data():
                if not rclpy.ok():
                    break
                if row is not None:
                    markers = self.build_marker_array(row)
                    objs = self.build_object_array(row)
                    marker_publisher.publish(markers)
                    object_publisher.publish(objs)

        except Exception as error:
            self.get_logger().error(str(error))
        finally:
            del riq
            self.get_logger().info("Stopped RadarIQ module")

    def build_marker(self, detection):
        color_idx = detection['tracking_id'] % len(self.colors)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "riq-object-markers"
        marker.id = detection['tracking_id']
        marker.header.frame_id = self.frame_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=1)  # Auto expire markers after 1 second unless they are updated
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.7
        marker.color.a = 1.0
        marker.color.r = float(self.colors[color_idx][0])
        marker.color.g = float(self.colors[color_idx][1])
        marker.color.b = float(self.colors[color_idx][2])
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = detection['x_pos']
        marker.pose.position.y = detection['y_pos']
        marker.pose.position.z = detection['z_pos']

        return marker

    def build_marker_array(self, riq_objects):
        markers = MarkerArray()
        for detection in riq_objects:
            marker = self.build_marker(detection)
            markers.markers.append(marker)
        return markers

    def build_object(self, detection):
        obj = RadarIQObject()
        obj.tracking_id = detection['tracking_id']
        obj.position.x = detection['x_pos']
        obj.position.y = detection['y_pos']
        obj.position.z = detection['z_pos']
        obj.velocity.x = detection['x_vel']
        obj.velocity.y = detection['y_vel']
        obj.velocity.z = detection['z_vel']
        obj.acceleration.x = detection['x_acc']
        obj.acceleration.y = detection['y_acc']
        obj.acceleration.z = detection['z_acc']
        return obj

    def build_object_array(self, riq_objects):
        objs = RadarIQObjects()
        objs.header.stamp = self.get_clock().now().to_msg()
        objs.header.frame_id = self.frame_id
        for detection in riq_objects:
            obj = self.build_object(detection)
            objs.objects.append(obj)
        return objs


def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    object_publisher = ObjectPublisher()
    rclpy.spin(object_publisher)

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
