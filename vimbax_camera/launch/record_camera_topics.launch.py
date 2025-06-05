#!/usr/bin/env python3
"""
Camera Topics Recording Launch File

This launch file starts recording all camera topics to a rosbag.
The rosbag is saved with a timestamp in the filename.
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create timestamp for the bag filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_filename = f'camera_system_{timestamp}'
    
    # Define the topics to record
    topics_to_record = [
        '/vimbax_camera_ir/camera_info',
        '/vimbax_camera_ir/image_raw',
        '/vimbax_camera_ir/image_raw/compressed',
        '/vimbax_camera_rgb/camera_info',
        '/vimbax_camera_rgb/image_raw',
        '/vimbax_camera_rgb/image_raw/compressed',
        '/wiris/thermal/camera_info',
        '/wiris/thermal/image',
        '/wiris/thermal/image/camera_info',
        '/wiris/thermal/image/compressed'
    ]
    
    # Create the rosbag record command
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_filename,
            *topics_to_record
        ],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        rosbag_record
    ]) 