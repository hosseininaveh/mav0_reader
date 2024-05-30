## MAV0 Sequence Publisher
EuRoC MAV dataset is a benchmarking dataset for monocular and stereo visual odometry that is captured from drone-mounted devices.

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
# Preparing the data
Start by downloading the dataset from here. 
http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/

Download the .zip file of a dataset you plan on using.

After downloading and uncompressing it, you will find several directories under the mav0/ directory. move this folder to mav0_reader/data/ folder
```
$ ls mav0/
body.yaml  cam0  cam1  imu0  leica0  state_groundtruth_estimate0

```

Make sure you have ROS2 installed and configured properly.

# Usage:

Launch ROS2:
```ros2 run mav0_reader mav0_reader```

The script will start publishing images, IMU data, UAV position, and state data. you can use rviz2 to see the data.

# Configuration:

Modify the file paths in the script to match the location of your MAV0 dataset.
Adjust sleep durations in the script if necessary to achieve the desired publishing rate.
Contributing:
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

# License:
MIT

# Related Publication

If you utilize MAV0 Reader, you might be interested in exploring the Visual SLAM algorithm. Among the most notable algorithms in this domain is ORB-SLAM 3. Our work focuses on enhancing the key frame selection component of this algorithm, incorporating photogrammetric principles within the PKS framework. Currently, I am engaged in adapting this algorithm from ROS 1 to ROS 2. I encourage you to review this algorithm and appropriately acknowledge it in your publication.
``` Azimi, A., Ahmadabadian, A.H. and Remondino, F., 2022. PKS: A photogrammetric key-frame selection method for visual-inertial systems built on ORB-SLAM3. ISPRS Journal of Photogrammetry and Remote Sensing, 191, pp.18-32.```
