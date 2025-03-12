# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense_ros2 px4_comm.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    
    # ros2 topic remap
    remappings = [
          ('imu', '/imu/data'),
          ('odom', '/camera/pose/sample'),]
    
    # vio forward to px4     
    vio_dummy_node = Node(
            package='realsense_ros2', 
            executable='vio_dummy.py', 
            output='screen')
    
    # vio forward to px4 
    vio_launch_path = os.path.join(
        get_package_share_directory('realsense_ros2'),
        'launch',
        'realsense_t265_odom.py'
    )
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vio_launch_path),
    )
    
    # vio forward to px4     
    vio_to_px_mavros_node = Node(
            package='realsense_ros2', 
            executable='vio_to_mavros_px4.py', 
            output='screen')
    
    # vio forward to px4     
    vio_to_px_dds_node = Node(
            package='realsense_ros2', 
            executable='vio_to_px4_dds.py', 
            remappings=remappings,
            output='screen')
    
    # mavros offboard control
    offboard_launch_path = os.path.join(
        get_package_share_directory('px4_offboard'),
        '',
        'offboard_control.launch.py'
    )
    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(offboard_launch_path),
    )
    
    return LaunchDescription([

        #vio_dummy_node,
        vio_launch,
        #vio_to_px_mavros_node,
        vio_to_px_dds_node,
        #offboard_launch
    ])
