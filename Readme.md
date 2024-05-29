## MAV0 Sequence Publisher

# Overview:
This Python script is designed to publish data from the MAV0 dataset using ROS2. It publishes images from two cameras, IMU data, UAV position, and state data.

# Prerequisites:

```
Python 3.x
ROS2 (Robot Operating System 2)
OpenCV
NumPy
Standard ROS2 packages (std_msgs, sensor_msgs, cv_bridge, rclpy, builtin_interfaces, tf2_ros, geometry_msgs, nav_msgs)
```
# Installation:

Clone this repository:
```
cd ros_ws/src
git clone https://github.com/hosseininaveh/mav0_reader.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build
```


Make sure you have ROS2 installed and configured properly.

Usage:

# Launch ROS2:
```ros2 run mav0_reader mav0_reader```

The script will start publishing images, IMU data, UAV position, and state data.

# Configuration:

Modify the file paths in the script to match the location of your MAV0 dataset.
Adjust sleep durations in the script if necessary to achieve the desired publishing rate.
Contributing:
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

# License:
MIT