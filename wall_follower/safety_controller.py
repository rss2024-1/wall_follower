#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("safety_controller")
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
        self.KP = 1.3
        self.KD = 3.5

	# TODO: Initialize your publishers and subscribers here
        self.subscription = self.create_subscription(
                LaserScan,
                self.SCAN_TOPIC,
                self.laser_callback,
                10)
        self.subscription 

        # self.actual_laser_subscriber = self.create_subscription(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd_mux/output', self.laser_callback, 10)
        # self.actual_laser_subscriber


        self.publisher = self.create_publisher(
                AckermannDriveStamped,
                self.DRIVE_TOPIC,
                10)
        
        # self.actual_publisher = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd_mux/input/safety', 10)

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
        if speed > 3.0:
            return control_output*(500)
        return control_output

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

        # Started adding from here!

        range_len = len(scan.ranges)

        low = -np.pi/3
        high = np.pi/3

        # estimate wall 
        num_ranges = np.array([float(i) for i in range(len(scan.ranges))])
        dist = np.array([float(i) for i in scan.ranges])
        angles = np.array([float(scan.angle_min + i*scan.angle_increment) for i in range(1+int((scan.angle_max - scan.angle_min)/scan.angle_increment))])
        
        mask = []
        for i in angles:
            if i > low and i<high: 
                mask.append(1)
            else:
                mask.append(0)  
                
        def frag(x, y): 
            idx = np.isfinite(y) & np.isfinite(x)
            m, b = np.polyfit(x[idx], y[idx], 1)
            wall_distance = abs(b*b/m)/(np.sqrt((b**2/m**2)+(b**2)))
            return wall_distance
        

        front_dist = np.ma.masked_array(dist, mask = mask)
        front_angles = np.ma.masked_array(angles, mask = mask)

        # front_y = np.multiply(front_dist, np.sin(front_angles))
        # front_x = np.multiply(front_dist, np.cos(front_angles))

        # front_distance = frag(front_x, front_y)
        front_distance = abs(np.mean(front_dist))

        drive_command = AckermannDriveStamped()
        if front_distance < 3: 
            drive_command.drive.speed = 0.0
            # self.get_logger().info(str(steer_msg.drive.speed))
            # self.publisher_.publish(steer_msg)
        else:
            drive_command.drive.speed = speed
        drive_command.header.stamp = rclpy.time.Time().to_msg()
        drive_command.header.frame_id = self.MAP_FRAME
        
        drive_command.drive.steering_angle = self.SIDE*-1*control_output
        drive_command.drive.steering_angle_velocity = 0.0
        
        drive_command.drive.acceleration = 0.0
        drive_command.drive.jerk = 0.0

        self.publisher.publish(drive_command)


def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()