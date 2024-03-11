#!/usr/bin/env python3   
import rclpy                                                                           
import numpy as np                                                                     
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
        # Fetch constants from the ROS parameter server                                        
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('stopping_distance').get_parameter_value().double_value
        self.MAP_FRAME = self.get_parameter('map_frame').get_parameter_value().string_value

        self.CURRENT_DRIVE = AckermannDriveStamped()
        self.CURRENT_DRIVE.header.stamp = rclpy.time.Time().to_msg()
        self.CURRENT_DRIVE.header.frame_id = self.MAP_FRAME
        self.CURRENT_DRIVE.drive.speed = 0.0
        self.CURRENT_DRIVE.drive.steering_angle = 0.0
        self.CURRENT_DRIVE.drive.steering_angle_velocity = 0.0

        self.CURRENT_DRIVE.drive.acceleration = 0.0
        self.CURRENT_DRIVE.drive.jerk = 0.0


        # TODO: Initialize your publishers and subscribers here
        self.subscription = self.create_subscription(
                LaserScan,
                "/scan",
                self.laser_callback,
                10)
        self.subscription

        self.publisher = self.create_publisher(AckermannDriveStamped, 'vesc/low_level/input/safety',10)

    def laser_callback(self, scan):
        self.STOPPING_DISTANCE = self.get_parameter('stopping_distance').get_parameter_value().double_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value

        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        divider_ix = int(len(ranges)/10)

        front_ranges = ranges[int(divider_ix*4.5):int(5.5*divider_ix)]
        front_angles = angles[divider_ix*4:6*divider_ix]

        close_count = (front_ranges < 1.3).sum()
        front_count = len(front_ranges)

        drive_command = AckermannDriveStamped()

        if close_count > (front_count/4):
            drive_command.header.stamp = rclpy.time.Time().to_msg()
            drive_command.header.frame_id = self.MAP_FRAME
            drive_command.drive.speed = 0.0
            drive_command.drive.acceleration = 0.0
            drive_command.drive.jerk = 0.0
            drive_command.drive.steering_angle = 0.0
            drive_command.drive.steering_angle_velocity = 0.0

            self.publisher.publish(drive_command)
        else:
            pass

def main():

    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()
 