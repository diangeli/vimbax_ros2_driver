#!/usr/bin/env python3
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare

DEFAULT_CONFIG_FOLDER = '/home/osar/drone_ws/src/vimbax_ros2_driver/vimbax_camera/params/params30_8_8_color'
ACTION_KEYS = {'device_key': 1, 'group_key': 1, 'group_mask': 1}
INTRA_PROCESS = [{'use_intra_process_comms': True}]
RECORDER_PARAMS = {
    'storage_id': 'mcap',
    'queue_frames': 1500,
    'image_topics': ['/vimbax_camera_rgb/image_raw', '/vimbax_camera_ir/image_raw'],
    'camera_info_topics': ['/vimbax_camera_rgb/camera_info', '/vimbax_camera_ir/camera_info'],
    'generic_topics': ['/wiris/thermal/image', '/wiris/thermal/camera_info',
                       '/wiris/thermal/image/camera_info'],
}


def camera_node(name, camera_id):
    return ComposableNode(
        package='vimbax_camera',
        plugin='vimbax_camera::VimbaXCameraNode',
        namespace=name,
        name=name,
        parameters=[{
            'camera_id': camera_id,
            'use_ros_time': ParameterValue(LaunchConfiguration('use_ros_time'), value_type=bool),
        }],
        extra_arguments=INTRA_PROCESS,
    )


def generate_launch_description():
    vimbax_camera_pkg = get_package_share_directory('vimbax_camera')
    wiris_pkg = get_package_share_directory('wiris_pro_driver')

    load_config = ExecuteProcess(
        cmd=[
            'python',
            os.path.join(vimbax_camera_pkg, 'scripts', 'load_camera_config.py'),
            '--settings_folder',
            PathJoinSubstitution([vimbax_camera_pkg, 'params', LaunchConfiguration('config_folder')]),
            '--trigger', LaunchConfiguration('trigger'),
            '--action-keys', *(str(key) for key in ACTION_KEYS.values()),
            '--trigger-rate-hz', LaunchConfiguration('trigger_rate_hz'),
            '--exposure-auto-max-us', LaunchConfiguration('exposure_auto_max_us'),
            '--ptp', LaunchConfiguration('ptp'),
        ],
        output='screen',
        name='load_camera_config'
    )

    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        sigterm_timeout='30',
        composable_node_descriptions=[
            camera_node('vimbax_camera_rgb', '192.168.1.40'),
            camera_node('vimbax_camera_ir', '192.168.1.45'),
            ComposableNode(
                package='vimbax_camera',
                plugin='vimbax_camera::ActionTriggerNode',
                name='action_trigger',
                parameters=[{
                    'rate_hz': ParameterValue(LaunchConfiguration('trigger_rate_hz'), value_type=float),
                    'interface_id': LaunchConfiguration('camera_interface'),
                    **ACTION_KEYS,
                }],
                extra_arguments=INTRA_PROCESS,
                condition=LaunchConfigurationEquals('trigger', 'action'),
            ),
            ComposableNode(
                package='vimbax_camera',
                plugin='vimbax_camera::SyncMonitorNode',
                name='sync_monitor',
                extra_arguments=INTRA_PROCESS,
            ),
            ComposableNode(
                package='camera_recorder',
                plugin='camera_recorder::RecorderNode',
                name='recorder',
                parameters=[{'output_dir': LaunchConfiguration('bag_dir'), **RECORDER_PARAMS}],
                extra_arguments=INTRA_PROCESS,
                condition=IfCondition(LaunchConfiguration('record')),
            ),
        ],
        output='screen',
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('drone_detection'), 'launch', 'detection_launch.py'])
        ),
        launch_arguments={
            'container': '/camera_container',
            'image_topic': '/vimbax_camera_rgb/image_raw',
            'model_path': LaunchConfiguration('model_path'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('detector')),
    )

    wiris_thermal = Node(
        package='wiris_pro_driver',
        executable='wiris_thermal_node',
        name='wiris_thermal_node',
        parameters=[os.path.join(wiris_pkg, 'params', 'camera_params.yaml')],
        output='screen'
    )

    def start_cameras_if_configured(event, context):
        if event.returncode == 0:
            return [camera_container, detector, wiris_thermal]
        reason = f'load_camera_config exited with code {event.returncode}; cameras not started'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_folder',
            default_value=DEFAULT_CONFIG_FOLDER,
            description='Folder (inside vimbax_camera share) containing camera configuration files'
        ),
        DeclareLaunchArgument('trigger', default_value='action'),
        DeclareLaunchArgument('trigger_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('camera_interface', default_value='eth0'),
        DeclareLaunchArgument('exposure_auto_max_us', default_value='20000'),
        DeclareLaunchArgument('ptp', default_value='slave'),
        DeclareLaunchArgument('use_ros_time', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value='~/bags'),
        DeclareLaunchArgument('detector', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=''),
        load_config,
        RegisterEventHandler(OnProcessExit(target_action=load_config, on_exit=start_cameras_if_configured)),
    ])
