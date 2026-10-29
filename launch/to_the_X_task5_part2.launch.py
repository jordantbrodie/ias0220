"""Task 5.2 - replay the lab drive to the X.

MuJoCo with world/drive_to_x.xml + our robot, the steering node, and playback of
the bag recorded in the lab (bags/rosbag_to_X).

ros2 launch ias0220_243483 to_the_X_task5_part2.launch.py
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
    mujoco_share = get_package_share_directory('dr_mujoco')
    mujoco_launch = os.path.join(mujoco_share, 'launch', 'default.launch.py')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=os.path.join(package_path, 'mujoco', 'my_robot.xml'),
        description='MuJoCo XML of our robot')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(package_path, 'bags', 'rosbag_to_X'),
        description='Bag recorded in the lab while driving to the X')

    # 1) MuJoCo with the drive_to_x world and our robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch),
        launch_arguments={
            'world': os.path.join(mujoco_share, 'world', 'drive_to_x.xml'),
            'robot': LaunchConfiguration('robot_model'),
        }.items())

    # 2) play the recording. The bag was recorded with -a, so it also contains the
    #    simulator's own topics (cmd_vel, odom, ...). Hint 4: only play the four
    #    sensor topics to avoid duplicates; the steering node recreates the commands.
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
             '--topics', '/imu', '/distance', '/tf', '/tf_static'],
        output='screen')

    # 3) our steering node turns the replayed sensor data into /diff_cont/cmd_vel
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
