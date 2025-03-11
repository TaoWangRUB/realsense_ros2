#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
import numpy as np
from time import time

class OdomToMavros(Node):
    def __init__(self):
        super().__init__('odom_to_mavros')
        
        # Subscriber to /odom
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers for MAVROS topics
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/mavros/vision_speed/speed_twist', 10)
        
        # Keep track of the last message received time
        self.is_publishing = False
        self.last_odom_time = time()  
        self.odom_timeout = 5.0  # Timeout threshold in seconds
        
        # Timer to check subscription status every 1 second
        self.timer = self.create_timer(1.0, self.check_odom_status)

    def odom_callback(self, msg):
    
        # directly forwarding
        if not self.is_publishing:
            self.get_logger().info('/odom will be forwareded to /mavros/vision_pose/pose and /mavros/vision_speed/speed_twist')
            self.is_publishing = True
        
        # Update last received time
        self.last_odom_time = time()
        
        # Extract pose and twist from /odom
        pose = msg.pose.pose
        twist = msg.twist.twist

        # Publish pose to /mavros/vision_pose/pose
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = pose
        self.pose_pub.publish(pose_msg)

        # Publish twist to /mavros/vision_speed/speed_twist
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = msg.header
        twist_msg.twist.twist = twist
        self.twist_pub.publish(twist_msg)
    
    def check_odom_status(self):
        # Check if /odom topic is still alive by checking the last received time
        if time() - self.last_odom_time > self.odom_timeout and self.is_publishing:
            self.get_logger().warning('/odom topic has not published for {:.2f} seconds.'.format(time() - self.last_odom_time))
            self.is_publishing = False
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = OdomToMavros()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
