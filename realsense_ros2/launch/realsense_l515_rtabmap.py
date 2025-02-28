# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    # Launch arguments
    unite_imu_method = DeclareLaunchArgument(
        'unite_imu_method', default_value='1',
        description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.')

    # Launch camera driver
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
            launch_arguments={'camera_namespace': '',
                              'enable_gyro': 'true',
                              'enable_accel': 'true',
                              'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                              'align_depth.enable': 'true',
                              'enable_sync': 'true',
                              'rgb_camera.profile': '640x480x30'}.items(),
    )

    vio_node = Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings)

    slam_node = Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'])

    rviz_node = Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings)
    
    ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("realsense_ros2"), 'params', 'ekf.yaml'),
                {"frequency": 50.0,
                 "predict_to_current_time": True,
                 "odom0": "/odom",
                 "odom0_config": [True, True, False,
                                  False, False, True,
                                  True, True, False,
                                  False, False, True,
                                  False, False, False],
                 "odom0_queue_size": 10,
                 "odom0_nodelay": False,
                 "imu0": "/imu/data",
                 "imu0_config": [False, False, False,
                                 True, True, True,
                                 False, False, False,
                                 True, True, True,
                                 True, True, True, ],
                 "imu0_queue_size": 10,
                 "imu0_nodelay": False,
                 "odom0_pose_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],  # Lower noise for odometry
                 "odom0_twist_noise": [0.01, 0.01, 0.01, 0.01, 0.01, 0.01],
                 "imu0_noise": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Higher noise for IMU
                }])
           
        # Compute quaternion of the IMU
    imu_filter_node = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':True,
                         'fixed_frame': "camera_link"}],
            remappings=[('imu/data_raw', '/camera/imu')])
    
    return LaunchDescription([
        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),
        
        SetParameter(name='Rtabmap/CameraRate', value=30),
        
        unite_imu_method,
        realsense_launch, 
        vio_node,
        slam_node,
        imu_filter_node,
        ekf_filter_node,
        rviz_node,
    ])
