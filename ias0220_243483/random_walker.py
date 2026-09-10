#!/usr/bin/env python3

import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class RandomWalker(Node):

    def __init__(self):
        super().__init__('walker')

        self.velocity_publisher = self.create_publisher(
            Vector3,
            'velocity',
            10
        )

        self.name_time_publisher = self.create_publisher(
            String,
            'name_and_time',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.timer_callback
        )

    def timer_callback(self):

        vx = random.choice([-1.0, 0.0, 1.0])
        vy = random.choice([-1.0, 0.0, 1.0])

        velocity_msg = Vector3()
        velocity_msg.x = vx
        velocity_msg.y = vy
        velocity_msg.z = 0.0

        self.velocity_publisher.publish(velocity_msg)

        info_msg = String()
        info_msg.data = f'243483,{time.time()}'

        self.name_time_publisher.publish(info_msg)

        self.get_logger().info(
            f'ID and time: {info_msg.data}'
        )

        self.get_logger().info(
            f'Velocity -> x:{vx}, y:{vy}, z:0'
        )


def main(args=None):

    rclpy.init(args=args)

    node = RandomWalker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
