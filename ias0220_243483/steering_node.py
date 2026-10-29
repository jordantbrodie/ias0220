#!/usr/bin/env python3
"""Task 5.2 - steering node: the sensor module becomes a joystick.

  /imu      (sensor_msgs/Imu)   -> pitch = throttle (linear.x), roll = steering (angular.z)
  /distance (sensor_msgs/Range) -> on-switch: drive only when something is close
  publishes /diff_cont/cmd_vel  (geometry_msgs/Twist) for the MuJoCo diff-drive robot
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from transforms3d.euler import quat2euler


class SteeringNode(Node):

    def __init__(self):
        super().__init__('steering_node')

        # Tunable parameters (can be changed with --ros-args -p name:=value)
        # Robot limit: wheel ctrlrange 10 rad/s * r 0.036 m = 0.36 m/s per wheel,
        # wheel base 0.35 m -> keep v + 0.175*w below 0.36 m/s.
        self.declare_parameter('max_linear_speed', 0.25)    # m/s
        self.declare_parameter('max_angular_speed', 0.6)    # rad/s
        self.declare_parameter('full_tilt_deg', 30.0)       # tilt that gives full speed
        self.declare_parameter('dead_zone_deg', 5.0)        # small tilts are ignored
        self.declare_parameter('distance_threshold', 0.30)  # m, farther than this = stop
        self.declare_parameter('publish_rate', 10.0)        # Hz
        self.declare_parameter('timeout', 0.5)              # s without data -> stop

        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.full_tilt = math.radians(self.get_parameter('full_tilt_deg').value)
        self.dead_zone = math.radians(self.get_parameter('dead_zone_deg').value)
        self.threshold = self.get_parameter('distance_threshold').value
        rate = self.get_parameter('publish_rate').value
        self.timeout = self.get_parameter('timeout').value

        # Latest sensor values (None = nothing received yet)
        self.roll = None
        self.pitch = None
        self.distance = None
        self.last_imu_time = None
        self.last_distance_time = None

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Range, '/distance', self.distance_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)

        # The sensor publishes at ~10 Hz (serial_interface timer = 0.1 s), so a
        # 10 Hz command rate uses every new reading without sending duplicates.
        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'Steering node started: {rate:.0f} Hz, max v={self.max_v} m/s, '
            f'max w={self.max_w} rad/s, distance threshold={self.threshold} m')

    def imu_callback(self, msg):
        q = msg.orientation
        # transforms3d wants quaternions as [w, x, y, z]; default axes 'sxyz'
        # give (roll about x, pitch about y, yaw about z) in radians.
        roll, pitch, _yaw = quat2euler([q.w, q.x, q.y, q.z])
        self.roll = roll
        self.pitch = pitch
        self.last_imu_time = self.get_clock().now()

    def distance_callback(self, msg):
        self.distance = msg.range
        self.last_distance_time = self.get_clock().now()

    def angle_to_speed(self, angle, max_speed):
        """Linear map with a dead zone: |angle| <= dead_zone -> 0, full_tilt -> max_speed."""
        if abs(angle) <= self.dead_zone:
            return 0.0
        scaled = (abs(angle) - self.dead_zone) / (self.full_tilt - self.dead_zone)
        speed = min(scaled, 1.0) * max_speed
        return math.copysign(speed, angle)

    def is_fresh(self, stamp):
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.timeout

    def object_in_front(self):
        # The sensor reports <= 0 (e.g. -0.01 m) when no echo came back,
        # so only a positive range below the threshold counts as "something there".
        return self.distance is not None and 0.0 < self.distance < self.threshold

    def timer_callback(self):
        cmd = Twist()   # all zeros = stop

        data_ok = self.is_fresh(self.last_imu_time) and self.is_fresh(self.last_distance_time)
        if data_ok and self.object_in_front():
            # Tilt forward (positive pitch, nose down) -> drive forward.
            # "Forward" = the IMU's x axis; on the module the ultrasonic sensor faces +y (left).
            cmd.linear.x = self.angle_to_speed(self.pitch, self.max_v)
            # Roll -> turning. Sign checked on the real module (held with its x axis
            # pointing away from the user): tilt right turns right, tilt left turns left.
            cmd.angular.z = self.angle_to_speed(self.roll, self.max_w)

        self.cmd_pub.publish(cmd)

        # Once per second, show what the node sees and what it commands
        if self.roll is not None and self.distance is not None:
            self.get_logger().info(
                f'roll={math.degrees(self.roll):6.1f} deg  pitch={math.degrees(self.pitch):6.1f} deg  '
                f'dist={self.distance:5.2f} m  ->  v={cmd.linear.x:5.2f}  w={cmd.angular.z:5.2f}',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # leave the robot stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
