from setuptools import setup
from glob import glob

package_name = 'radariq_ros_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('rviz/*.urdf')),
        ('share/' + package_name, glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'radariq'],
    zip_safe=True,
    maintainer='RadarIQ',
    maintainer_email='support@radariq.io',
    description='The RadarIQ ROS Package, supports the RIQ-M model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = radariq_ros_driver.pointcloud_publisher_node:main',
            'objects_publisher = radariq_ros_driver.objects_publisher_node:main',
            'example_application = radariq_ros_driver.example_application:main',
        ],
    },
)
