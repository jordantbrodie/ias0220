"""Task 7 - PD position/orientation control.

Starts MuJoCo with my robot (empty world), a static transform map -> odom,
the PD controller (simple_control.py, gains from config/simple_control_v2.yaml)
and RViz showing odometry and the waypoints.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg_share = get_package_share_directory('ias0220_243483')
    dr_mujoco_share = get_package_share_directory('dr_mujoco')

    robot_path = os.path.join(my_pkg_share, 'mujoco', 'my_robot.xml')
    # Empty plane: the default drive_to_x world has obstacles near the path
    world_path = os.path.join(dr_mujoco_share, 'world', 'default.xml')
    config = os.path.join(my_pkg_share, 'config', 'simple_control_v2.yaml')
    rviz_config = os.path.join(my_pkg_share, 'config', 'differential_robot_rviz_task7.rviz')

    # Simulator: publishes /diff_cont/odom, listens on /diff_cont/cmd_vel
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dr_mujoco_share, 'launch', 'default.launch.py')
        ),
        launch_arguments={'robot': robot_path, 'world': world_path}.items(),
    )

    # The sim publishes no TF; tie RViz's fixed frame 'map' to 'odom' (identity)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    # PD controller; node name 'controller' matches the yaml's top-level key
    controller = Node(
        package='ias0220_243483',
        executable='simple_control.py',
        name='controller',
        output='screen',
        parameters=[config],
        remappings=[('/diff_cont/cmd_vel', '/diff_cont/cmd_vel')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([simulation, static_tf, controller, rviz])
