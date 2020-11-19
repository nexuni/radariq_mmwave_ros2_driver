import launch
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_dir = get_package_share_directory('radariq_ros_driver')
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'radariq_objects.launch.py')))

    urdf_path = os.path.join(base_dir, 'radariq_sensor.urdf')
    sensor_model = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    rviz_config = os.path.join(base_dir, 'radariq_objects.rviz')
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return launch.LaunchDescription([rviz_node, sensor_model, included_launch])
