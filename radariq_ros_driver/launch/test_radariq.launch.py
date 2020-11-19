import launch
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_dir = get_package_share_directory('radariq_ros_driver')
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(base_dir, 'radariq_pointcloud.launch.py')))

    exampe_application = Node(
        package='radariq_ros_driver',
        node_executable='example_application',
    )

    return launch.LaunchDescription([exampe_application, included_launch])
