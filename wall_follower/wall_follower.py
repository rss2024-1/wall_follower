#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import os
import time
from wall_follower.visualization_tools import VisualizationTools
import csv

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")
        self.declare_parameter("map_frame", "default")
        self.declare_parameter("max_speed", "default")
        self.declare_parameter("max_steering_angle", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.MAP_FRAME = self.get_parameter('map_frame').get_parameter_value().string_value
        
        self.prev_error = 0.0 
        self.KP = 2.5
        self.KD = 35.0

        data_dir = "data"
        laser_scan_data_dir = "pid_error"

        # Check if the data directory exists, if not create it
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)

        # Check if the laser_scan_data directory exists inside the data directory, if not create it
        if not os.path.exists(os.path.join(data_dir, laser_scan_data_dir)):
            os.makedirs(os.path.join(data_dir, laser_scan_data_dir))
    
        self.data_file_name = f"{data_dir}/{laser_scan_data_dir}/pid_error_{str(time.time())}.csv"


	# TODO: Initialize your publishers and subscribers here
        self.subscription = self.create_subscription(
                LaserScan,
                self.SCAN_TOPIC,
                self.laser_callback,
                10)
        self.subscription 

        self.publisher = self.create_publisher(
                AckermannDriveStamped,
                self.DRIVE_TOPIC,
                10)
        
        self.data_dump = self.create_publisher(String, "data_dump", 10)

        self.line_pub = self.create_publisher(Marker, self.SCAN_TOPIC, 1)
        
    # TODO: Write your callback functions here    
    def calculate_error(self, wall_y):
        distance = wall_y[5*len(wall_y)//8]
        error = self.DESIRED_DISTANCE - np.absolute(distance)
        return error    

    def pd_controller(self, error, speed):
        error_deriv = error - self.prev_error
        self.prev_error = error
        control_output = self.KP*error + self.KD*error_deriv
        if speed > 3.85:
            return control_output*(5)
        return control_output
    
    def pid_error_to_csv(self, scan, error, steering_angle):
        seconds = scan.header.stamp.sec
        nanoseconds = scan.header.stamp.nanosec

        timestamp = seconds + nanoseconds/(10**9)

        # Check if file exists to decide whether to write headers
        file_exists = os.path.isfile(self.data_file_name)

        with open(self.data_file_name, 'a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["Timestamp", "error", "desired_distance_from_wall", "side", "velocity", "Kp", "Kd", "steering_angle"])
            writer.writerow([timestamp, error, self.DESIRED_DISTANCE, self.SIDE, self.VELOCITY, self.KP, self.KD, steering_angle])

    def laser_callback(self, scan):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        ranges = np.array(scan.ranges)
        divider_ix = int(len(ranges)/3)

        right_scans = ranges[3*divider_ix//5:7*divider_ix//5]
        left_scans = ranges[8*divider_ix//5:12*divider_ix//5]
        scans = np.array([])

        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)

        if self.SIDE < 0:
            scans = right_scans
            angles = angles[3*divider_ix//5:7*divider_ix//5]
        else:
            scans = left_scans
            angles = angles[8*divider_ix//5:12*divider_ix//5]

        x = scans*np.cos(angles)
        y = scans*np.sin(angles)
        m, y_int = np.linalg.lstsq(np.vstack([x, np.ones(len(x))]).T, y, rcond=None)[0]


        wall_y = m*x+y_int
        VisualizationTools.plot_line(x, wall_y, self.line_pub, frame="/laser")

        speed = min(self.VELOCITY, 4.0)
        error = self.calculate_error(wall_y)

        control_output = self.pd_controller(error, speed)
        control_output = min(control_output, .32)
        #self.get_logger().info('Speed: % f' % speed)

        drive_command = AckermannDriveStamped()
        drive_command.header.stamp = rclpy.time.Time().to_msg()
        drive_command.header.frame_id = self.MAP_FRAME

        drive_command.drive.steering_angle = self.SIDE*-1*control_output
        drive_command.drive.steering_angle_velocity = 0.0
        drive_command.drive.speed = speed
        drive_command.drive.acceleration = 0.0
        drive_command.drive.jerk = 0.0

        self.publisher.publish(drive_command)

        self.pid_error_to_csv(scan, error, drive_command.drive.steering_angle)

def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
