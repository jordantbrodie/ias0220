#!/usr/bin/env python3
"""Task 4.2 - wheel odometry from the dr_mujoco encoder ticks."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import qmult

# Robot geometry (must match mujoco/my_robot.xml)
WHEEL_RADIUS = 0.036       # m  (72 mm wheel diameter / 2)
WHEEL_SEPARATION = 0.35    # m  (track: distance between wheel centres)

# Encoder settings (from dr_mujoco simulation.py)
COUNTS_PER_REV = 508.8                 # used to convert counts to radians
WRAP_PERIOD = int(COUNTS_PER_REV)      # 508: dr_mujoco wraps with "% int(cpr)"
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')

        # Latest encoder counts and whether each one has arrived yet
        self.left_count = 0
        self.right_count = 0
        self.left_received = False
        self.right_received = False

        # Values from the previous odometry update
        self.prev_left = 0
        self.prev_right = 0
        self.prev_time = None
        self.initialized = False

        # Robot pose in the odom frame
        self.x = 0.0
        self.y = 0.0
        self.q = [1.0, 0.0, 0.0, 0.0]   # transforms3d order: [w, x, y, z]

        # Subscribers: one per wheel encoder
        self.create_subscription(Int32, '/left_encoder/ticks', self.left_callback, 10)
        self.create_subscription(Int32, '/right_encoder/ticks', self.right_callback, 10)

        # Publisher and the message we keep updating
        self.odom_pub = self.create_publisher(Odometry, '/my_odom', 10)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_body'
        self.odom_msg.pose.pose.orientation.w = 1.0   # identity quaternion

        # Odometry update at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def left_callback(self, msg):
        self.left_count = msg.data
        self.left_received = True

    def right_callback(self, msg):
        self.right_count = msg.data
        self.right_received = True

    @staticmethod
    def count_difference(current, previous):
        """Change in counts, corrected for the encoder wrapping around."""
        diff = current - previous
        if diff > WRAP_PERIOD / 2:
            diff -= WRAP_PERIOD
        elif diff < -WRAP_PERIOD / 2:
            diff += WRAP_PERIOD
        return diff

    def timer_callback(self):
        # Wait until both encoders have sent data
        if not (self.left_received and self.right_received):
            return

        now = self.get_clock().now()
        left = self.left_count
        right = self.right_count

        # First reading: just remember it as the starting point
        if not self.initialized:
            self.prev_left = left
            self.prev_right = right
            self.prev_time = now
            self.initialized = True
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # 1. Encoder counts -> distance travelled by each wheel
        d_left = self.count_difference(left, self.prev_left) * RAD_PER_COUNT * WHEEL_RADIUS
        d_right = -self.count_difference(right, self.prev_right) * RAD_PER_COUNT * WHEEL_RADIUS
        # (right encoder counts the opposite way, so its distance is negated)

        # 2. Wheel distances -> robot forward distance and heading change
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEEL_SEPARATION

        # 3. Velocities in the robot (base_body) frame
        self.odom_msg.twist.twist.linear.x = d_center / dt
        self.odom_msg.twist.twist.angular.z = d_theta / dt

        # 4. Update orientation: rotate the old quaternion by d_theta around z
        q_step = euler2quat(0.0, 0.0, d_theta)
        self.q = qmult(self.q, q_step)
        yaw = quat2euler(self.q)[2]

        # 5. Update position with the heading halfway through the movement
        mid_yaw = yaw - d_theta / 2.0
        self.x += d_center * math.cos(mid_yaw)
        self.y += d_center * math.sin(mid_yaw)

        # 6. Fill in and publish the message
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.w = self.q[0]
        self.odom_msg.pose.pose.orientation.x = self.q[1]
        self.odom_msg.pose.pose.orientation.y = self.q[2]
        self.odom_msg.pose.pose.orientation.z = self.q[3]
        self.odom_msg.header.stamp = now.to_msg()
        self.odom_pub.publish(self.odom_msg)

        # Remember this update for next time
        self.prev_left = left
        self.prev_right = right
        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
