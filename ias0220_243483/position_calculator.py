#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose


class PositionCalculator(Node):

    def __init__(self):
        super().__init__('position_calculator')

        self.position = Pose()

        self.subscription_time = self.create_subscription(
            String,
            'name_and_time',
            self.time_callback,
            10
        )

        self.subscription_velocity = self.create_subscription(
            Vector3,
            'velocity',
            self.velocity_callback,
            10
        )

    def time_callback(self, msg):

        student_id, timestamp = msg.data.split(',')

        self.get_logger().info(
            f'Student {student_id} contacted me, '
            f'and told me that the current time is: {timestamp}'
        )

    def velocity_callback(self, msg):

        dt = 0.5

        self.position.position.x += msg.x * dt
        self.position.position.y += msg.y * dt
        self.position.position.z = 0.0

        self.get_logger().info(
            'The new position of the walker is:\n'
            f'x = {self.position.position.x}\n'
            f'y = {self.position.position.y}\n'
            f'z = {self.position.position.z}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionCalculator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
