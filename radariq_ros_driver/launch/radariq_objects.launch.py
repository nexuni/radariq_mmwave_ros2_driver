from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pointcloud_node = Node(
        package="radariq_ros_driver",
        node_executable="objects_publisher",
        parameters=[
            {"serial_port": "/dev/radariq"},
            {"frame_id": "radar"},
            {"object_data_topic": "radariq_objects"},
            {"marker_topic": "radariq_markers"},
            {"mode": 1},
            {"framerate": 10},
            {"distancefilter_min": 0.0},
            {"distancefilter_max": 10.0},
            {"anglefilter_min": -45},
            {"anglefilter_max": 45},
            {"pointdensity": 0},
            {"certainty": 5}
        ]
    )
    ld.add_action(pointcloud_node)
    return ld
