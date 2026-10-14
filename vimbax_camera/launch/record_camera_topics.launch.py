#!/usr/bin/env python3
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_system_launch import INTRA_PROCESS, RECORDER_PARAMS  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('container', default_value='/camera_container'),
        DeclareLaunchArgument('bag_dir', default_value='~/bags'),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_recorder',
                    plugin='camera_recorder::RecorderNode',
                    name='recorder',
                    parameters=[{'output_dir': LaunchConfiguration('bag_dir'), **RECORDER_PARAMS}],
                    extra_arguments=INTRA_PROCESS,
                ),
            ],
        ),
    ])
