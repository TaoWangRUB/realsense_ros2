from setuptools import setup
import os
from glob import glob

package_name = 'realsense_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=['src.python.vio_to_px4'],  # Make sure this matches the file structure
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tao Wang',
    maintainer_email='wtlove876@gmail.com',
    description='ROS 2 package for RealSense VIO integration with PX4',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vio_to_px4 = src.python.vio_to_px4:main'  
            'vio_to_mavros_px4 = src.python.vio_to_mavros_px4:main'  
            'vio_dummy = src.python.vio_dummy:main'  
        ],
    },
)

