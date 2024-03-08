#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("stopping_distance", "default")
        self.declare_parameter("map_frame", "default")

        ### NEW ####
        # added this extra distance checker to 'double-check' if it should actually stop
        self.declare_parameter("checking_distance", "default")
        # scales the distance from wall to be less than it actually is to account for movement
        self.declare_parameter("time_safety_scalar", "default")
        ############

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('stopping_distance').get_parameter_value().double_value
        self.MAP_FRAME = self.get_parameter('map_frame').get_parameter_value().string_value

        ### NEW ###
        self.CHECKING_DISTANCE = self.get_parameter('checking_distance').get_parameter_value().double_value
        self.TIME_SAFETY_SCALAR = self.get_parameter('time_safety_scalar').get_parameter_value().double_value
        ###########

        # REAL ROBOT publisher/subscribers
        self.subscription = self.create_subscription(
                LaserScan,
                self.SCAN_TOPIC,
                self.laser_callback,
                10)
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd', self.drive_callback, 10)
        self.safety_publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)

    def laser_callback(self, scan):

        ### NEW ####
        # calculate some dt
        time_now = rclpy.time.Time()
        dt = time_now-time_before
        time_before = rclpy.time.Time()

        # get checking distance and time scalar for later calculation
        self.CHECKING_DISTANCE = self.get_parameter('checking_distance').get_parameter_value().double_value
        self.TIME_SAFETY_SCALAR = self.get_parameter('time_safety_scalar').get_parameter_value().double_value
        ############

        # get current speed and desired stopping distance
        self.STOPPING_DISTANCE = self.get_parameter('stopping_distance').get_parameter_value().double_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value

        # distance and angle scans
        ranges = np.array(scan.ranges)
        divider_ix = int(len(ranges)/10) 
        front_ranges = ranges[divider_ix*4:6*divider_ix]
        # angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        # front_angles = angles[divider_ix*4:6*divider_ix]
        
        # speed alteration logic
        drive_command = AckermannDriveStamped() # message template for drive cmd

        #### NEW #######
        # gives a buffer; checks if current readings pass a 'warning threshold'
        # stops if we're really going to crash
        close_points_to_check = [dist for dist in front_ranges if dist < self.STOPPING_DISTANCE]
        closest_dist = min(close_points_to_check)
        if closest_dist < self.CHECKING_DISTANCE:
            predicted_closest_reading = closest_dist - self.VELOCITY*self.TIME_SAFETY_SCALAR*dt
            if predicted_closest_reading <= self.STOPPING_DISTANCE:
                drive_command.drive.speed = 0.0
        ################
        
        # close_count = (front_ranges < self.STOPPING_DISTANCE).sum()
        # if close_count > 3: 
        #     drive_command.drive.speed = 0.0
        self.safety_publisher.publish(drive_command)

def main():
    
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()