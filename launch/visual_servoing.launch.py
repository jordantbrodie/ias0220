"""Task 8.1 - visual servoing (following a stationary ball).

Starts the course simulation (dr_mujoco, branch 'visual_servoing') with MY robot,
a static transform map -> front_camera_frame and RViz showing the camera image.
The visual_servoing node is added in a later step.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Folder where colcon installed each package (install/<pkg>/share/<pkg>)
    my_pkg_share = get_package_share_directory('ias0220_243483')
    dr_mujoco_share = get_package_share_directory('dr_mujoco')

    robot_path = os.path.join(my_pkg_share, 'mujoco', 'my_robot.xml')
    rviz_config = os.path.join(my_pkg_share, 'config', 'visual_servoing.rviz')

    # Course simulation with the default world (blue ball), my robot passed via 'robot'.
    # The camera 'front_camera' in my_robot.xml is published on /camera1/image_raw.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dr_mujoco_share, 'launch', 'default.launch.py')
        ),
        launch_arguments={'robot': robot_path}.items(),
    )

    # Task 8.1 node: finds the blue ball and drives to it with a PD controller.
    # Publishes /diff_cont/cmd_vel, /ball_mask and /ball_tracking.
    # Final tuned values are the node's defaults (kp_v 3.0, kd_v 0.2, kp_w 2.0, kd_w 0.1,
    # x* 0.5, d* 0.4); they can be overridden here with parameters=[{...}] if needed.
    visual_servoing = Node(
        package='ias0220_243483',
        executable='visual_servoing.py',
        name='visual_servoing',
        output='screen',
    )

    # The simulator publishes no TF, but the images have frame_id 'front_camera_frame'.
    # This fixed (identity) transform map -> front_camera_frame clears RViz's fixed-frame warning.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_front_camera',
        arguments=['--frame-id', 'map', '--child-frame-id', 'front_camera_frame'],
        output='screen',
    )

    # RViz with an Image display on /camera1/image_raw (use_sim_time stays false: no /clock).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        simulation,
        visual_servoing,
        static_tf,
        rviz,
    ])
