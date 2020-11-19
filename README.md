# How to get started with Radar IQ and ROS2.

Note: These instructions have been tested with ROS dashing but should work with other version of ROS.

Some prior knowledge about how ROS2 works is assumed in this tutorial. Newcomers to ROS2 should check out the getting started guide on the [offical ROS wiki](https://index.ros.org/doc/ros2/).

![RViz Screenshot](docs/rviz.png "RViz Screenshot")

In this tutorial you will learn how to:
1. Configure Linux so that the RadarIQ module is presented consistently as a device.
2. Setup a new ROS workspace for the RadarIQ module.
3. Visualize the data using RViz.
4. Configure settings and incorporate the module into an existing ROS project.

# Configure Linux
By default USB devices are presented to the Linux Kernel as /dev/ttyACMxx (where xx is a number).
The exact device name changes depending on the order in which the USB devices are connected or detected by the kernel.

In order to make USB devices easier to use, it is advisable to ensure they are always mapped to a common system name.

RadarIQ comes with a script for making the RadarIQ module present itself as /dev/radariq.

## Running the script
1. Change to the radariq_ros package.

   ``cd radariq_ros``

2. Make the setup scripts executable.

   ``chmod +x setup.sh``

3. Run the setup script and choose the "Install" option

   ```./setup.sh```

4. Plug the RadarIQ module into the ROS machine and check that it was correctly detected.

   ``ls /dev | grep "radariq"``

If the RadarIQ module was correctly detected ``radariq`` should be shown.


# Setup a new ROS workspace for RadarIQ
Note: These instructions assume your workspace is named ``my_ws``. If your workspace is named differently, adjust accordingly.

1. Create a new workspace as per the [getting started instructions on the ROS website](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/).

2.  Make sure you are in the my_ws directory and have sourced the setup script for the workspace.

   ``cd ~/my_ws``
   
   ``source /opt/ros/<distribution>/setup.sh``
 
   ``.  install/local_setup.sh``
   
3. Clone the RadarIQ ROS repository into the src directory.

   ``git clone https://github.com/radariq/ros2 ./src/radariq_ros``

4. Install the RadarIQ pip package
   
   ``pip3 install radariq``

5. Run calcon to make the workspace.

   ``colcon build``
   
6. Re-initialize the local overlay

   ``.  install/local_setup.sh``

# Visualize the data using RViz
RadarIQ pointcloud data can be started and visualised using the ROS RViz tool.

1. From your ROS workspace run

   ``ros2 launch radariq_ros_driver view_radariq_pointcloud.launch.py``

The RadarIQ module should start up and RViz should open, showing a point cloud of detections.

2. From your ROS workspace run

   ``ros2 launch radariq_ros_driver view_radariq_objects.launch``

The RadarIQ module should start up and RViz should open, showing a object representations of the detections.

# Incorporate into an existing ROS project
These instructions explain how to incorporate RadarIQ into an existing ROS workspace using a provided sample application
 which simply echos the data out to the ROS log.

## Steps

1. Adjust the module settings

   The module settings such as distance filters, angle filters etc can adjusted using a launch file.

   The default launch file is located: ``src/radariq_ros/radariq_ros_driver/launch/radariq.launch``

2. Start ROS core

   ``roscore``

3. Start the publisher node

   ``rosrun radariq_ros_driver publisher_node.py``

4. Start the test application

   ``rosrun radariq_ros_driver test_radariq.launch``

Raw data should be echoed out to the ROS log (see ``example_application.py`)
