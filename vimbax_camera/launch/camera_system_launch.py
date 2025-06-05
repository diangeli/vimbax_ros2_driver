#!/usr/bin/env python3
"""
Camera System Launch File

This launch file orchestrates the initialization and execution of multiple camera nodes
in a specific sequence to ensure proper system operation. It handles:
1. Camera configuration loading
2. VimbaX camera initialization (IR and RGB)
3. Multi-camera synchronization
4. WIRIS thermal camera integration
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

# Default folder name (can be overridden on the command line)
DEFAULT_CONFIG_FOLDER = '/home/osar/drone_ws/src/vimbax_ros2_driver/vimbax_camera/params/params30_8_8_color'
SYNC_TIME_THRESHOLD = '1'  # in milliseconds

def generate_launch_description():
    # 1. Declare the config_folder argument so users can override it:
    config_folder_arg = DeclareLaunchArgument(
        'config_folder',
        default_value=DEFAULT_CONFIG_FOLDER,
        description='Folder (inside vimbax_camera share) containing camera configuration files'
    )

    # 2. Compute absolute paths to the two packages
    vimbax_camera_pkg = get_package_share_directory('vimbax_camera')
    wiris_pkg = get_package_share_directory('wiris_pro_driver')

    # 3. ExecuteProcess that runs the Python config script
    #    (We assume you've moved load_camera_config.py into 'scripts/' in the installed share directory,
    #     or you have installed it as a console script. Here we'll assume scripts/)
    load_config = ExecuteProcess(
        cmd=[
            'python',
            os.path.join(vimbax_camera_pkg, 'scripts', 'load_camera_config.py'),
            '--settings_folder',
            PathJoinSubstitution([
                vimbax_camera_pkg,
                'params',
                LaunchConfiguration('config_folder')
            ])
        ],
        output='screen',
        name='load_camera_config'
    )

    # 4. Once load_config finishes, start all camera nodes.
    #    We’ll wrap everything else in an OnProcessExit handler so it only launches after load_config is done.
    # 4a. Include the VimbaX camera launch (passing the same config_folder)
    vimbax_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vimbax_camera_pkg, 'launch', 'vimbax_camera_launch.py')
        ),
        launch_arguments={'config_folder': LaunchConfiguration('config_folder')}.items()
    )

    # 4b. Multi-camera synchronization node (ensure the 'executable' name is spelled exactly as installed)
    sync_node = Node(
        package='vimbax_camera',
        executable='multi_camera_synchronisation',   # <-- check exact spelling in your CMakeLists
        name='multi_camera_sync',
        arguments=[SYNC_TIME_THRESHOLD],
        output='screen',
        parameters=[{'use_ros_time': True}]
    )

    # 4c. WIRIS thermal camera node, pointing at a YAML in share/wiris_pro_driver/params/
    wiris_thermal = Node(
        package='wiris_pro_driver',
        executable='wiris_thermal_node',
        name='wiris_thermal_node',
        parameters=[os.path.join(wiris_pkg, 'params', 'camera_params.yaml')],
        output='screen'
    )

    # 5. Use an event handler so that everything in "camera_group" only starts after load_config exits
    camera_group = [vimbax_launch, sync_node, wiris_thermal]
    sequenced_camera_group = RegisterEventHandler(
        OnProcessExit(
            target_action=load_config,
            on_exit=camera_group
        )
    )

    # 6. Build the LaunchDescription
    ld = LaunchDescription()
    # 6a. Add the launch‐argument first
    ld.add_action(config_folder_arg)
    # 6b. Kick off load_config immediately
    ld.add_action(load_config)
    # 6c. Only once load_config completes, spawn the rest
    ld.add_action(sequenced_camera_group)

    return ld
