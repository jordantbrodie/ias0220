"""Task 5.2 - ONLINE: MuJoCo robot + real sensor module (serial_interface) + steering node.

ros2 launch ias0220_243483 differential_robot_sensors_steering_task5_part2_online_data.launch.py
Optional: serial_port:=/dev/ttyACM0   world:=<path to world xml>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(mujoco_share, 'world', 'drive_to_x.xml'),
        description='MuJoCo world')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of the sensor module (e.g. /dev/ttyUSB0 or /dev/ttyACM0)')

    # 1) the simulated robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch),
        launch_arguments={
            'robot': LaunchConfiguration('robot_model'),
            'world': LaunchConfiguration('world'),
        }.items())

    # 2) the provided driver: reads USB, publishes /imu, /distance and the sensor TF
    serial_interface = Node(
        package='ias0220_sensors',
        executable='serial_interface',
        name='sensors',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')}])

    # map -> imu_link, as in the ias0220_sensors launch, so the sensor TF tree is complete
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'imu_link'])

    # 3) our steering node
    steering = Node(
        package='ias0220_243483',
        executable='steering_node.py',
        name='steering_node',
        output='screen')

    return LaunchDescription([
        robot_model_arg,
        world_arg,
        serial_port_arg,
        simulation,
        serial_interface,
        imu_static_tf,
        steering,
    ])
