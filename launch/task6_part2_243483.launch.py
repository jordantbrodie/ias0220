"""Task 6.2 - object tracking.

Starts the course's object detection world (MuJoCo + ball_mover) with MY robot,
the object_recognition node, a static transform map -> front_camera_frame and RViz
showing the camera image, the processed image (/ball_tracking) and the mask.
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
    rviz_config = os.path.join(my_pkg_share, 'config', 'task6_part2_243483.rviz')

    # Course world (simulation + ball_mover), with my robot passed via the 'robot' argument.
    # The camera 'front_camera' in my_robot.xml is published on /camera1/image_raw.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dr_mujoco_share, 'launch', 'object_detection_world.launch.py')
        ),
        launch_arguments={'robot': robot_path}.items(),
    )

    # Finds the red ball: publishes /ball_mask and /ball_tracking
    object_recognition = Node(
        package='ias0220_243483',
        executable='object_recognition.py',
        name='object_recognition',
        output='screen',
    )

    # Hint 3: the simulator publishes no TF, but the images have frame_id 'front_camera_frame'.
    # This fixed (identity) transform map -> front_camera_frame clears RViz's fixed-frame error.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_front_camera',
        arguments=['--frame-id', 'map', '--child-frame-id', 'front_camera_frame'],
        output='screen',
    )

    # Hint 2: use_sim_time stays false (default) - nothing publishes /clock.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        simulation,
        object_recognition,
        static_tf,
        rviz,
    ])
