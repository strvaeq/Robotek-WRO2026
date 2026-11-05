"""Open Challenge: hardware layer plus the Open controller node.

    ros2 launch renkay_bringup open.launch.py

Press the RRC Lite button once to calibrate and start, once more to stop.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HARDWARE_ARGS = ('use_sim_time', 'params_file', 'lidar_port', 'video_device',
                 'motor_driver_package')


def _node(context):
    params = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    params_file = LaunchConfiguration('params_file').perform(context)
    if params_file:
        params.insert(0, params_file)
    return [Node(
        package='renkay_open',
        executable='open_node',
        name='open_challenge',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('renkay_bringup'),
            'launch', 'bringup.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name in HARDWARE_ARGS}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock. false on the real car.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional ROS parameter file. The Open controller reads '
                        'its tuning from config.py, so config/open.yaml is reference only.'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/renkay_lidar',
            description='Serial device of the LD19 LiDAR.'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='V4L2 device of the USB camera.'),
        DeclareLaunchArgument(
            'motor_driver_package', default_value='renkay_open',
            description='Package that provides the motor_driver executable.'),
        hardware,
        OpaqueFunction(function=_node),
    ])
