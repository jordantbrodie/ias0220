"""Replay a provided or recorded ROS bag with the saved sensor RViz layout."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_playback(context):
    bag = Path(LaunchConfiguration('bag_path').perform(context)).expanduser().resolve()
    if not bag.exists():
        raise RuntimeError(f'Bag not found: {bag}. Pass bag_path:=/path/to/bag1 or record the lab sensor first.')
    config = str(Path(get_package_share_directory('ias0220_243483')) / 'config' / 'task5_part1.rviz')
    # Bags already contain TF. Do not start live sensor or duplicate transform publishers.
    return [
        ExecuteProcess(cmd=['ros2', 'bag', 'play', str(bag), '--loop', '--clock'], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='bags/recorded', description='ROS bag folder or file'),
        OpaqueFunction(function=start_playback),
    ])
