# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense_ros2 l515_slam_toolbox.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    # Path to the RealSense launch file
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    
    # Launch arguments
    unite_imu_method = DeclareLaunchArgument(
        'unite_imu_method', default_value='2',
        description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.')
    
    # Include RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={'camera_namespace': '',
                          'enable_gyro': 'true',
                          'enable_accel': 'true',
                          'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                          'align_depth.enable': 'true',
                          'enable_sync': 'true',
                          'pointcloud.enable': 'false',
                          'rgb_camera.profile': '640x480x30'}.items(),
    )
    
    # Launch slam_toolbox as a Node (not IncludeLaunchDescription)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': 'true',
            'use_imu': True,  # Enable IMU integration
            'use_odom': True,
            'base_frame': 'camera_link',
            'odom_frame': 'odom',
            'map_frame': 'map'
        }],
        remappings=[
            ('/imu', '/imu/data')  # Remap IMU topic
        ]
    )
    
    # depth to laserscan node
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        node_executable='depthimage_to_laserscan_node',
        node_name='scan',
        output='screen',
        parameters=[{'output_frame':'camera_link'}],
        remappings=[('depth','/camera/aligned_depth_to_color/image_raw'),
                    ('depth_camera_info', '/camera/aligned_depth_to_color/camera_info')],
    )
    
    # Pointcloud to laserscan node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        parameters=[{
            'target_frame': 'camera_link',  # Target frame for the laser scan
            'transform_tolerance': 0.01,    # Transform tolerance
            'min_height': 0.0,              # Minimum height to include in the scan
            'max_height': 1.0,              # Maximum height to include in the scan
            'angle_min': -3.14159,          # Minimum angle (radians)
            'angle_max': 3.14159,           # Maximum angle (radians)
            'angle_increment': 0.0087,      # Angle increment (radians)
            'scan_time': 0.1,               # Scan time (seconds)
            'range_min': 0.1,               # Minimum range (meters)
            'range_max': 10.0,              # Maximum range (meters)
            'use_inf': True,                # Use infinity for out-of-range measurements
        }],
        remappings=[
            ('cloud_in', '/camera/pointcloud'),  # Remap input topic
            ('scan', '/scan')                    # Remap output topic
        ],
        output='screen'
    )
    
    # Compute quaternion of the IMU
    imu_filter_node = Node(
        package='imu_filter_madgwick', 
        executable='imu_filter_madgwick_node', 
        output='screen',
        parameters=[{'use_mag': False, 
                     'world_frame':'enu', 
                     'publish_tf':True,
                     'fixed_frame': "camera_link"}],
        remappings=[('imu/data_raw', '/camera/imu')])
    
    static_tf_camera_link_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'base_link']
    )
    
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False}]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/aligned_depth_to_color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]
    
    # vio by using rtabmp      
    vio_node = Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings)
    
    # Robot Localization (fused odom)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('realsense_ros2'), 'config', 'ekf.yaml')]
    )
    
    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'slam_toolbox_default.rviz')]
    )

    return LaunchDescription([
        unite_imu_method,  # Add the DeclareLaunchArgument to the LaunchDescription
        realsense_launch,
        #static_tf_camera_link_to_base_link, 
        depth_to_laserscan_node,
        imu_filter_node,
        vio_node,
        slam_toolbox_node,
        rviz2_node
    ])
