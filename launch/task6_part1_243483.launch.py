"""Task 6.1 - camera calibration.

Starts the image publisher, the calibration node, image_proc's RectifyNode (inside a
component container) and RViz showing /image_raw, /image_processed and /image_rect.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('ias0220_243483'),
                               'config', 'task6_part1_243483.rviz')

    # Plays the calibration photos on /image_raw at 2 Hz
    image_publisher = Node(
        package='ias0220_243483',
        executable='image_publisher.py',
        name='image_publisher',
        output='screen',
    )

    # Collects corners, calibrates, then publishes /camera_info for every image
    camera_calibration = Node(
        package='ias0220_243483',
        executable='camera_calibration.py',
        name='camera_calibration',
        output='screen',
    )

    # On Jazzy image_proc has no executable: RectifyNode is a component (plugin)
    # that must be loaded into a container process.
    # Its default topics are image, camera_info and image_rect (checked with ros2 node info).
    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image', '/image_raw'),
                    ('camera_info', '/camera_info'),
                    ('image_rect', '/image_rect'),
                ],
            ),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        image_publisher,
        camera_calibration,
        rectify_container,
        rviz,
    ])
