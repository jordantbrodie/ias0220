"""Task 4.1 - spawn the differential-drive robot in MuJoCo, drive it with the
keyboard and inspect it in RViz and rqt_graph."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Provided simulator package (not modified) and its default launch file
    mujoco_pkg_share = get_package_share_directory('dr_mujoco')
    simulation_launch_path = os.path.join(
        mujoco_pkg_share, 'launch', 'default.launch.py')

    # Our own package: robot model and RViz config are installed here
    package_path = get_package_share_directory('ias0220_243483')
    rvizconfig = os.path.join(package_path, 'config', 'task4_part1.rviz')

    # Path of the robot XML; can be overridden on the command line
    robot_model_path_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=os.path.join(package_path, 'mujoco', 'my_robot.xml'),
        description='Absolute path to the MuJoCo XML robot model')

    # Start the MuJoCo simulation, passing our model as its "robot" argument
    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path),
        launch_arguments={
            'robot': LaunchConfiguration('robot_model'),
        }.items())

    # Keyboard teleop in its own xterm; commands go to the diff-drive controller
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_input',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', 'diff_cont/cmd_vel')])

    # map -> odom: identity, since there is no localization yet
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', rvizconfig],
        output='screen')

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph')

    return LaunchDescription([
        robot_model_path_arg,
        simulation_include,
        teleop_node,
        tf2_node,
        rviz2_node,
        rqt_graph_node,
    ])
