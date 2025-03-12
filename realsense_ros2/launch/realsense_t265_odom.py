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

from ament_index_python.packages import get_package_share_directory

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
                              'rgb_camera.profile': '640x480x30',
                              'publish_odom_tf': 'true',
                              'publish_tf': 'true'}.items(),
    )
    
    # base_link to camera_link
    tf_base_to_pose = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_baselink_cameraPose',
        arguments = ["0.1", "0", "-0.15", "0", "0", "0", "camera_link", "base_link"],
        output="screen"
    )
    
    # odom remapping
    tf_odom_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_cameraOdom',
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "odom_frame"],
        output="screen"
    )
    
    return LaunchDescription([
        # make launch argument available
        unite_imu_method,
        # start t265
        realsense_launch, 
        # overwrite tf base to pose
        tf_base_to_pose,
        # overwrite tf odom to odom
        tf_odom_to_odom
    ])
