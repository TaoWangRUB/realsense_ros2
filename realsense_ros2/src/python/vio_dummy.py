#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class VioDummy(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for /odom topic
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Timer to publish data at a fixed rate
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Parameters for circular motion
        self.radius = 10.0
        self.yaw_rate = 0.0
        self.x = self.radius  # Initial position
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

    def timer_callback(self):
        # Get the current time
        current_time = self.get_clock().now().to_msg()

        # Simulate circular motion
        self.yaw += self.yaw_rate * 0.1
        self.x = self.radius * math.cos(self.yaw)
        self.y = self.radius * math.sin(self.yaw)

        # Publish map → odom transform (static)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0  # Static translation
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0  # Static rotation (no rotation)
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(map_to_odom)

        # Publish odom → base_link transform (dynamic)
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = current_time
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'
        odom_to_base_link.transform.translation.x = self.x
        odom_to_base_link.transform.translation.y = self.y
        odom_to_base_link.transform.translation.z = self.z
        odom_to_base_link.transform.rotation.z = math.sin(self.yaw / 2)
        odom_to_base_link.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_broadcaster.sendTransform(odom_to_base_link)

        # Publish base_link → base_link_frd transform (static)
        base_link_to_base_link_frd = TransformStamped()
        base_link_to_base_link_frd.header.stamp = current_time
        base_link_to_base_link_frd.header.frame_id = 'base_link'
        base_link_to_base_link_frd.child_frame_id = 'base_link_frd'
        base_link_to_base_link_frd.transform.translation.x = 0.0  # Static translation
        base_link_to_base_link_frd.transform.translation.y = 0.0
        base_link_to_base_link_frd.transform.translation.z = 0.0
        base_link_to_base_link_frd.transform.rotation.x = 0.0  # Static rotation (no rotation)
        base_link_to_base_link_frd.transform.rotation.y = 0.0
        base_link_to_base_link_frd.transform.rotation.z = 0.0
        base_link_to_base_link_frd.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(base_link_to_base_link_frd)

        # Publish /odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link_frd'  # Use base_link_frd for PX4
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z  # Ensure z is set to a stable value
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom_msg.twist.twist.linear.x = 0.0  # Linear velocity (optional)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0  # Angular velocity (optional)
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VioDummy()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
