"""Hardware layer of the Renkay car: RRC Lite board, LiDAR, camera, motors.

Starts the four hardware nodes and nothing else. The challenge controller is
added by open.launch.py or obstacle.launch.py, which include this file, so
each challenge can be started with one command.

    ros2 launch renkay_bringup bringup.launch.py

Every device path is a launch argument. No absolute path is hardcoded.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _hardware(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    motor_params = [params_file, sim_time] if params_file else [sim_time]

    return [
        # RRC Lite board: IMU, button, buzzer, steering servo.
        Node(
            package='ros_robot_controller',
            executable='ros_robot_controller',
            name='ros_robot_controller',
            output='screen',
            parameters=[sim_time],
        ),
        # LD19 / STL-19P LiDAR, 230400 baud, publishes /scan.
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': 'scan',
                'frame_id': 'base_laser',
                'port_name': LaunchConfiguration('lidar_port'),
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
            }, sim_time],
        ),
        # USB camera, publishes /usb_cam/image_raw (used by the Obstacle node).
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': 'usb_cam',
            }, sim_time],
        ),
        # TB6612FNG motor driver: subscribes /motor_vel.
        # Both controller packages ship this node; the default copy is renkay_open.
        Node(
            package=LaunchConfiguration('motor_driver_package'),
            executable='motor_driver',
            name='tb6612fng_motor',
            output='screen',
            parameters=motor_params,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock. false on the real car.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional ROS parameter file for the motor driver. '
                        'Empty = the driver defaults declared in motor_driver.py. '
                        'The challenge node does not read ROS parameters yet: its '
                        'tuning lives in config.py, so config/*.yaml is reference only.'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/renkay_lidar',
            description='Serial device of the LD19 LiDAR (udev name, or /dev/ttyUSB0).'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='V4L2 device of the USB camera.'),
        DeclareLaunchArgument(
            'motor_driver_package', default_value='renkay_open',
            description='Package that provides the motor_driver executable: '
                        'renkay_open (default) or renkay_obstacle.'),
        OpaqueFunction(function=_hardware),
    ])
