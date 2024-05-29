import threading
from std_msgs.msg import Header
from sensor_msgs.msg import Image, Imu  
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import rclpy
from rclpy.node import Node
import time
import csv  
from builtin_interfaces.msg import Time
import tf2_ros 
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry  # Import Odometry message
import numpy as np

class Mav0SequencePublisher(Node):

    def __init__(self):
        super().__init__('mav0_sequence_publisher')
        
        # Initialize publishers
        self.image_pub_cam0 = self.create_publisher(Image, 'cam0/image_raw', 10)
        self.image_pub_cam1 = self.create_publisher(Image, 'cam1/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/imu_data', 10)  
        self.position_pub = self.create_publisher(PoseStamped, 'uav/position', 10)
        self.odometry_pub = self.create_publisher(Odometry, 'uav/odometry', 10)  # New publisher for odometry
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Directory paths
        self.cam0_data_csv = '/home/ali/ros_ws/src/mav0_reader/data/mav0/cam0/data.csv'
        self.cam1_data_csv = '/home/ali/ros_ws/src/mav0_reader/data/mav0/cam1/data.csv'
        self.cam0_dir = '/home/ali/ros_ws/src/mav0_reader/data/mav0/cam0/data/'
        self.cam1_dir = '/home/ali/ros_ws/src/mav0_reader/data/mav0/cam1/data/'
        self.imu_data_csv = '/home/ali/ros_ws/src/mav0_reader/data/mav0/imu0/data.csv'
        self.position_data_csv = '/home/ali/ros_ws/src/mav0_reader/data/mav0/leica0/data.csv'
        self.state_data_csv = '/home/ali/ros_ws/src/mav0_reader/data/mav0/state_groundtruth_estimate0/data.csv'  # State data CSV
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Track total processing time and count of processed images
        self.total_processing_time = 0
        self.num_processed_images = 0
        
        # Initialize state attributes
        self.last_imu_timestamp = None
        self.integrated_angular_velocity = np.zeros(3)
        self.integrated_linear_acceleration = np.zeros(3)
        self.quaternion = [1, 0, 0, 0]  # Initial quaternion
        self.translation = [0, 0, 0]  # Initial translation

        self.get_logger().info('Starting publisher...')
        self.image_publisher_thread = threading.Thread(target=self.publish_images_loop)
        self.image_publisher_thread.start()
        
        # Start publishing loop for IMU data
        self.imu_publisher_thread = threading.Thread(target=self.publish_imu_data_loop)
        self.imu_publisher_thread.start()
        
        # Start publishing loop for UAV position data
        self.position_publisher_thread = threading.Thread(target=self.publish_position_data_loop)
        self.position_publisher_thread.start()
        
        # Start publishing loop for state data
        self.state_publisher_thread = threading.Thread(target=self.publish_state_data_loop)
        self.state_publisher_thread.start()
    
    def publish_images(self):
        # Read the timestamps from CSV files
        timestamps_cam0 = self.read_timestamps_from_csv(self.cam0_data_csv)
        timestamps_cam1 = self.read_timestamps_from_csv(self.cam1_data_csv)

        png_paths0 = sorted([os.path.join(self.cam0_dir, f) for f in os.listdir(self.cam0_dir) if f.endswith('.png')])
        png_paths1 = sorted([os.path.join(self.cam1_dir, f) for f in os.listdir(self.cam1_dir) if f.endswith('.png')])

        # Iterate over the collected paths to read, process, and publish the images
        for img_path0, img_path1, ts0, ts1 in zip(png_paths0, png_paths1, timestamps_cam0, timestamps_cam1):
            start_time = time.time()
            img0 = cv2.imread(img_path0)
            img1 = cv2.imread(img_path1)
            end_time = time.time()

            # Update total processing time and count of processed images
            self.total_processing_time += end_time - start_time
            self.num_processed_images += 1

            # Create headers for the image messages using timestamps from CSV
            header0 = Header(stamp=self.convert_timestamp_to_ros_time(ts0), frame_id='cam0_frame')
            header1 = Header(stamp=self.convert_timestamp_to_ros_time(ts1), frame_id='cam1_frame')

            # Calculate average processing time
            avg_processing_time = self.total_processing_time / self.num_processed_images if self.num_processed_images > 0 else 0

            # Adjust sleep duration to ensure 30Hz rate
            sleep_duration = max(1/30 - avg_processing_time, 0.05)
            time.sleep(sleep_duration)

            try:
                with self.lock:  # Use lock to ensure thread safety
                    image_msg0 = self.bridge.cv2_to_imgmsg(img0, "bgr8")
                    image_msg0.header = header0
                    image_msg1 = self.bridge.cv2_to_imgmsg(img1, "bgr8")
                    image_msg1.header = header1
                    self.image_pub_cam0.publish(image_msg0)
                    self.image_pub_cam1.publish(image_msg1)

            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Error: {str(e)}")

    def read_timestamps_from_csv(self, csv_file):
        timestamps = []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                timestamps.append(int(row[0]))
        return timestamps

    def convert_timestamp_to_ros_time(self, timestamp_ns):
        ros_time = Time()
        ros_time.sec = timestamp_ns // 1_000_000_000
        ros_time.nanosec = timestamp_ns % 1_000_000_000
        return ros_time

    def euler_to_quaternion(self, angles, delta_time):
        theta = np.linalg.norm(angles) * delta_time
        if theta == 0:
            return [1, 0, 0, 0]  # No rotation

        k = np.array([np.cos(theta / 2), np.sin(theta / 2)])
        j = np.cross(k, angles)
        j /= np.linalg.norm(j)
        i = np.cross(j, angles)
        i /= np.linalg.norm(i)
        return np.concatenate((i, j, k))

    def integrate_imu_data(self, imu_msg):
        if not hasattr(self, 'last_imu_timestamp'):
            self.last_imu_timestamp = imu_msg.header.stamp.nanosec
            self.integrated_angular_velocity = [0, 0, 0]
            self.integrated_linear_acceleration = [0, 0, 0]
            self.quaternion = [1, 0, 0, 0]  # Initial quaternion
            self.translation = [0, 0, 0]  # Initial translation

        # Compute delta time in seconds 
        delta_time = (imu_msg.header.stamp.nanosec - self.last_imu_timestamp) / 1e9

        omega = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
        self.integrated_angular_velocity = np.add(self.integrated_angular_velocity, omega * delta_time)

        a = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
        self.integrated_linear_acceleration = np.add(self.integrated_linear_acceleration, a * delta_time)

        self.last_imu_timestamp = imu_msg.header.stamp.nanosec

        q = self.euler_to_quaternion(self.integrated_angular_velocity, delta_time)
        self.quaternion = q

        self.translation = self.integrated_linear_acceleration

        return self.quaternion, self.translation

    def csv_to_imu(self, csv_file):
        imu_msgs = []
        prev_timestamp_ns = None  

        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                timestamp_ns = int(row[0])
                w_RS_S_x = float(row[1])
                w_RS_S_y = float(row[2])
                w_RS_S_z = float(row[3])
                a_RS_S_x = float(row[4])
                a_RS_S_y = float(row[5])
                a_RS_S_z = float(row[6])

                imu_msg = Imu()
                imu_msg.orientation_covariance = [-1, 0, 0, 0, -1, 0, 0, 0, -1]
                imu_msg.angular_velocity.x = w_RS_S_x
                imu_msg.angular_velocity.y = w_RS_S_y
                imu_msg.angular_velocity.z = w_RS_S_z

                imu_msg.linear_acceleration.x = a_RS_S_x
                imu_msg.linear_acceleration.y = a_RS_S_y
                imu_msg.linear_acceleration.z = a_RS_S_z

                header = Header()
                header.stamp = self.convert_timestamp_to_ros_time(timestamp_ns)
                header.frame_id = 'imu_link'
                imu_msg.header = header

                if prev_timestamp_ns is not None:
                    delta_time = (timestamp_ns - prev_timestamp_ns) / 1e9
                else:
                    delta_time = 0.0000000001

                prev_timestamp_ns = timestamp_ns

                omega = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
                self.integrated_angular_velocity = np.add(self.integrated_angular_velocity, omega * delta_time)

                a = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
                self.integrated_linear_acceleration = np.add(self.integrated_linear_acceleration, a * delta_time)

                self.last_imu_timestamp = imu_msg.header.stamp.nanosec

                q = self.euler_to_quaternion(self.integrated_angular_velocity, delta_time)
                self.quaternion = q

                self.translation = self.integrated_linear_acceleration
                imu_msg.orientation.x = self.quaternion[0]
                imu_msg.orientation.y = self.quaternion[1]
                imu_msg.orientation.z = self.quaternion[2]
                imu_msg.orientation.w = self.quaternion[3]

                imu_msgs.append(imu_msg)
        return imu_msgs

    def publish_imu_data(self, imu_dir):
        imu_msgs = self.csv_to_imu(imu_dir)
        for imu_msg in imu_msgs:
            self.imu_pub.publish(imu_msg)
            quaternion, translation = self.integrate_imu_data(imu_msg)
            self.publish_tf("map", "imu_link", translation, quaternion)
            time.sleep(0.05)

    def read_position_data(self, csv_file):
        positions = []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                timestamp_ns = int(row[0])
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
                positions.append((timestamp_ns, x, y, z))
        return positions

    def publish_position_data(self, position_data):
        for timestamp_ns, x, y, z in position_data:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.convert_timestamp_to_ros_time(timestamp_ns)
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = self.quaternion[0]
            pose_msg.pose.orientation.y = self.quaternion[1]
            pose_msg.pose.orientation.z = self.quaternion[2]
            pose_msg.pose.orientation.w = self.quaternion[3]

            self.position_pub.publish(pose_msg)
            time.sleep(0.05)
    
    def read_state_data(self, csv_file):
        states = []
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                timestamp_ns = int(row[0])
                p_RS_R_x = float(row[1])
                p_RS_R_y = float(row[2])
                p_RS_R_z = float(row[3])
                q_RS_w = float(row[4])
                q_RS_x = float(row[5])
                q_RS_y = float(row[6])
                q_RS_z = float(row[7])
                v_RS_R_x = float(row[8])
                v_RS_R_y = float(row[9])
                v_RS_R_z = float(row[10])
                states.append((timestamp_ns, p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z, v_RS_R_x, v_RS_R_y, v_RS_R_z))
        return states

    def publish_state_data(self, state_data):
        for state in state_data:
            timestamp_ns, p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z, v_RS_R_x, v_RS_R_y, v_RS_R_z = state
            
            odom_msg = Odometry()
            odom_msg.header.stamp = self.convert_timestamp_to_ros_time(timestamp_ns)
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = p_RS_R_x
            odom_msg.pose.pose.position.y = p_RS_R_y
            odom_msg.pose.pose.position.z = p_RS_R_z
            odom_msg.pose.pose.orientation.w = q_RS_w
            odom_msg.pose.pose.orientation.x = q_RS_x
            odom_msg.pose.pose.orientation.y = q_RS_y
            odom_msg.pose.pose.orientation.z = q_RS_z

            odom_msg.twist.twist.linear.x = v_RS_R_x
            odom_msg.twist.twist.linear.y = v_RS_R_y
            odom_msg.twist.twist.linear.z = v_RS_R_z

            self.odometry_pub.publish(odom_msg)
            time.sleep(0.05)

    def publish_images_loop(self):
        while rclpy.ok():
            self.publish_images()

    def publish_imu_data_loop(self):
        while rclpy.ok():
            self.publish_imu_data(self.imu_data_csv)
    
    def publish_position_data_loop(self):
        position_data = self.read_position_data(self.position_data_csv)
        while rclpy.ok():
            self.publish_position_data(position_data)
    
    def publish_state_data_loop(self):
        state_data = self.read_state_data(self.state_data_csv)
        while rclpy.ok():
            self.publish_state_data(state_data)
    
    def publish_tf(self, source_frame, target_frame, translation, quaternion):
        quaternion = [float(q) for q in quaternion]
        tf_msg = self.get_tf_message(source_frame, target_frame, translation, quaternion)
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def get_tf_message(self, source_frame, target_frame, translation, quaternion):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = source_frame
        tf_msg.child_frame_id = target_frame
        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]
        return tf_msg                     

def main(args=None):
    rclpy.init(args=args)
    publisher = Mav0SequencePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
    publisher.image_publisher_thread.join()  # Ensure threads are properly shut down
    publisher.imu_publisher_thread.join()
    publisher.position_publisher_thread.join()
    publisher.state_publisher_thread.join()  # Ensure state publisher thread is properly shut down

if __name__ == '__main__':
    main()

