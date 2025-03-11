#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
import numpy as np
from time import time

class VIOPublisher(Node):
    def __init__(self):
        super().__init__('vio_publisher')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.px4_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        
        self.is_publishing = False
        # Keep track of the last message received time
        self.last_odom_time = time()  
        self.odom_timeout = 5.0  # Timeout threshold in seconds
        
        # Timer to check subscription status every 1 second
        self.timer = self.create_timer(1.0, self.check_odom_status)

    def odom_callback(self, msg):
        # directly forwarding
        if not self.is_publishing:
            self.get_logger().info('/odom will be forwareded to /fmu/in/vehicle_odometry')
            self.is_publishing = True
        
        # Update last received time
        self.last_odom_time = time()
        
        vio_msg = VehicleOdometry()
        vio_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 expects timestamps in microseconds
        vio_msg.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        vio_msg.q = [q.x, q.y, q.z, q.w]  # PX4 uses [x, y, z, w] order

        # Set velocity (if available)
        vio_msg.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        vio_msg.angular_velocity = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        vio_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED  # PX4 uses NED frame

        self.px4_pub.publish(vio_msg)
    
    def check_odom_status(self):
        # Check if /odom topic is still alive by checking the last received time
        if time() - self.last_odom_time > self.odom_timeout and self.is_publishing:
            self.get_logger().warning('/odom topic has not published for {:.2f} seconds.'.format(time() - self.last_odom_time))
            self.is_publishing = False
        else:
            pass

def main():
    rclpy.init()
    node = VIOPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

