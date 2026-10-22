"""Record the lab sensor; run from the package folder so bags/recorded is in the submission."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from serial.tools import list_ports


def start_recording(context):
    port = LaunchConfiguration('serial_port').perform(context)
    if port == 'auto':
        candidates = sorted(p.device for p in list_ports.comports()
                            if p.device.startswith(('/dev/ttyUSB', '/dev/ttyACM')))
        if len(candidates) != 1:
            raise RuntimeError('Connect one lab sensor, or specify serial_port:=/dev/ttyUSB0. '
                               f'Detected ports: {candidates}')
        port = candidates[0]
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError(f'Cannot read/write {port}. Check connection and dialout membership.')
    output = Path(LaunchConfiguration('output_bag').perform(context)).expanduser().resolve()
    if output.exists():
        raise RuntimeError(f'{output} already exists. Preserve it and choose output_bag:=bags/recorded_take2.')
    output.parent.mkdir(parents=True, exist_ok=True)
    config = str(Path(get_package_share_directory('ias0220_243483')) / 'config' / 'task5_part1.rviz')
    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'map', '--child-frame-id', 'imu_link']),
        Node(package='ias0220_sensors', executable='serial_interface', output='screen',
             parameters=[{'serial_port': port}]),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', str(output)], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config], output='screen',
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='auto', description='auto, /dev/ttyUSB0 or /dev/ttyACM0'),
        DeclareLaunchArgument('output_bag', default_value='bags/recorded', description='New recording folder; never overwritten'),
        DeclareLaunchArgument('show_rviz', default_value='true', description='Open RViz while recording'),
        OpaqueFunction(function=start_recording),
    ])
