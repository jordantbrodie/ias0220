"""Task 5.2 - OFFLINE: MuJoCo robot + recorded sensor bag + steering node.

ros2 launch ias0220_243483 differential_robot_rosbag_steering_task5_part2_offline_data.launch.py
Optional: bag_path:=/some/other/bag
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('ias0220_243483')
    mujoco_launch = os.path.join(
        get_package_share_directory('dr_mujoco'), 'launch', 'default.launch.py')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=os.path.join(package_path, 'mujoco', 'my_robot.xml'),
        description='MuJoCo XML of our robot')

    # bag2 from the course (3 pitch rotations with object at ~12 cm, 3 at ~20 cm, 3 with nothing)
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(package_path, 'bags', 'record2'),
        description='Bag folder to play back')

    # 1) the simulated robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch),
        launch_arguments={'robot': LaunchConfiguration('robot_model')}.items())

    # 2) ros2 bag play (same command as the ias0220_sensors template: rate 1, loop)
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1', '-l',
             LaunchConfiguration('bag_path'),
             '--topics', '/imu', '/distance', '/tf', '/tf_static'],
        output='screen')

    # 3) our steering node, replacing teleop_twist_keyboard
    steering = Node(
        package='ias0220_243483',
        executable='steering_node.py',
        name='steering_node',
        output='screen')

    return LaunchDescription([
        robot_model_arg,
        bag_path_arg,
        simulation,
        bag_play,
        steering,
    ])
