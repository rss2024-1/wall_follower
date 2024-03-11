#!/usr/bin/env python3   
import rclpy                                                                           
import numpy as np                                                                     
from rclpy.node import Node           
from sensor_msgs.msg import LaserScan                                                  
import csv
import os
import pandas as pd
import time as time


### 1. Read the existing data into a pandas dataframe, df1
### 2. Put the new data into a new pandas dataframe, df2
### 3. Use pd.concat to concatenate the two, making df3
### 4. Save it, via df3.to_csv()


class LaserScanData(Node):                                                                                                                                                     
    def __init__(self):                                                                        
        super().__init__("listen_range_data")       

        # try:
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        # except:
            # self.SCAN_TOPIC = "/scan"

        self.subscription = self.create_subscription(
                LaserScan,
                 self.SCAN_TOPIC,
                self.laser_callback,
                10)

        data_dir = "data"
        laser_scan_data_dir = "laser_scan_data"

        # Check if the data directory exists, if not create it
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)

        # Check if the laser_scan_data directory exists inside the data directory, if not create it
        if not os.path.exists(os.path.join(data_dir, laser_scan_data_dir)):
            os.makedirs(os.path.join(data_dir, laser_scan_data_dir))
    
        self.data_file_name = f"{data_dir}/{laser_scan_data_dir}/laser_scan_data_{str(time.time())}.csv"
        # self.data_file_name = f"laser_scan_data_{str(time.time())}.csv"
        
    def laser_callback(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)

        seconds = scan.header.stamp.sec
        nanoseconds = scan.header.stamp.nanosec

        timestamp = seconds + nanoseconds/(10**9)

        # Check if file exists to decide whether to write headers
        file_exists = os.path.isfile(self.data_file_name)

        with open(self.data_file_name, 'a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["Timestamp", "Angle Increment", "Time Increment", "Angle Min", "Angle Max", "Range", "Angle"])
            for i in range(len(ranges)):
                writer.writerow([timestamp, scan.angle_increment, scan.time_increment, scan.angle_min, scan.angle_max, ranges[i], angles[i]])

def main():
    rclpy.init()
    Laser_Scan_Data = LaserScanData()
    rclpy.spin(Laser_Scan_Data)
    Laser_Scan_Data.destroy_node()
    rclpy.shutdown()
 