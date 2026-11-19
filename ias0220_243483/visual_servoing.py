#!/usr/bin/env python3
"""Task 8.1 - visual servoing: drive to a stationary blue ball using only the camera.

  subscribes /camera1/image_raw  (sensor_msgs/Image, 640x480 from the dr_mujoco camera)
  publishes  /diff_cont/cmd_vel  (geometry_msgs/Twist: v = linear.x, omega = angular.z)
             /ball_mask          (mono8 Image: white = pixel has the ball's colour)
             /ball_tracking      (bgr8 Image: camera image + detected circle + desired x*, d*)

Image features (normalised by the image WIDTH, so both are 0..1):
  x_n = horizontal position of the ball centre / width   (0 = left edge, 1 = right edge)
  d_n = ball diameter / width                             (bigger = closer)

Errors and PD control (image-based visual servoing):
  e_w = x* - x_n     ball left of centre -> x_n < 0.5 -> e_w > 0 -> omega > 0 = turn LEFT
  e_v = d* - d_n     ball too small (far) -> e_v > 0   -> v > 0     = drive FORWARD
  v     = kp_v * e_v + kd_v * de_v/dt
  omega = kp_w * e_w + kd_w * de_w/dt
  de/dt = (e - e_previous) / dt   (backward difference, dt from the image time stamps)

No ball in the image -> v = omega = 0 and the derivative history is reset
(Task 8.2 will rotate to search instead, see handle_no_ball()).

All limits and gains are ROS parameters, tune live with:
  ros2 param set /visual_servoing <name> <value>
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class VisualServoing(Node):

    # Robot geometry (from mujoco/my_robot.xml), used to keep the wheels below their limit
    TRACK_WIDTH = 0.35          # distance between the wheels [m]
    MAX_WHEEL_SPEED = 0.36      # 10 rad/s (actuator ctrlrange) * 0.036 m wheel radius [m/s]

    def __init__(self):
        super().__init__('visual_servoing')

        # HSV limits for the BLUE ball, measured on a camera frame (min/median/max):
        #   ball : H 112/120/120  S 174/230/231  V 68/151/218
        #   sky  : H 105          S 142-146      V 114-140
        #   floor: H 104-106      S 126-173      V 62-87
        # -> h_min 110 is above the sky/floor hue (<=106), s_min 150 is above the sky,
        #    so BOTH hue and saturation keep the background out.
        self.declare_parameter('h_min', 110)
        self.declare_parameter('h_max', 130)
        self.declare_parameter('s_min', 150)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 30)    # shadowed underside of the ball is darker
        self.declare_parameter('v_max', 255)

        # Desired feature values (where the ball should end up in the image)
        self.declare_parameter('x_desired', 0.5)   # x*: ball centre in the middle of the image
        self.declare_parameter('d_desired', 0.4)   # d*: ball diameter = 40% of the image width

        # PD gains, tuned live with 'ros2 param set' + ball teleport trials (/cmd_pos/ball):
        #   kp_v 1.5: smooth but braked from ~2.3 m and stalled at e_v ~0.011 (command 0.016 m/s
        #             is too weak to beat wheel friction) -> never entered the arrived zone
        #   kp_v 3.0: full speed until ~1.5 m, ~3.5 s braking, 0 overshoot, exact stop (v = w = 0)
        #   Tested: ball 26.6 deg left / 6.7 m, 20 deg left / 4 m, 30 deg right / 5 m.
        #   kd_v damps the braking (no overshoot), kp_w/kd_w: turn in ~1.5 s, no left/right wobble.
        self.declare_parameter('kp_v', 3.0)    # [m/s per unit of e_v]
        self.declare_parameter('kd_v', 0.2)
        self.declare_parameter('kp_w', 2.0)    # [rad/s per unit of e_w]
        self.declare_parameter('kd_w', 0.1)

        # Output limits (clamps)
        # (named max_* so they do not clash with the HSV limit 'v_max' = maximum brightness)
        self.declare_parameter('max_linear_speed', 0.35)    # [m/s]   wheels can do 0.36 m/s at most
        self.declare_parameter('max_angular_speed', 1.0)    # [rad/s]

        # "Arrived" zone: if BOTH errors are this small, send exactly v = omega = 0.
        # Needed because tiny commands (< ~0.014 m/s) cannot beat the wheel friction
        # (kv 0.05 * 0.4 rad/s = frictionloss 0.02), so the robot would stand still while
        # the controller keeps asking for "a tiny bit more" forever.
        self.declare_parameter('d_tolerance', 0.01)   # +-1% of image width (~ +-3 cm)
        self.declare_parameter('x_tolerance', 0.02)   # +-2% of image width (~ +-1.5 deg)

        # Contours smaller than this (in pixels^2) are treated as noise (same as Task 6.2)
        self.min_area = 5.0

        # Derivative history: errors and time of the previous image with a ball
        self.prev_e_v = None
        self.prev_e_w = None
        self.prev_time = None

        self.bridge = CvBridge()   # converts ROS Image <-> OpenCV image (numpy array)

        self.image_sub = self.create_subscription(
            Image, '/camera1/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)
        self.mask_pub = self.create_publisher(Image, '/ball_mask', 10)
        self.tracking_pub = self.create_publisher(Image, '/ball_tracking', 10)

        self.get_logger().info('visual_servoing started, waiting for /camera1/image_raw')

    # ------------------------------------------------------------------ main loop
    def image_callback(self, msg):
        self.current_header = msg.header

        # 1. ROS Image -> OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        # 2. Find the ball
        ball = self.detect_ball(frame)

        if ball is None:
            self.handle_no_ball()
        else:
            # 3. Features, normalised by the image width (Hint) -> 0..1
            cx, cy, radius = ball
            x_n = cx / width
            d_n = (2.0 * radius) / width

            # 4. PD control -> velocity command
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # [s]
            v, w = self.compute_control(x_n, d_n, stamp)
            self.publish_cmd(v, w)
            self.get_logger().info(
                f'x_n={x_n:.3f} d_n={d_n:.3f} -> v={v:.3f} m/s  w={w:.3f} rad/s',
                throttle_duration_sec=0.5)

        # 5. Debug image (every frame, also without a ball)
        self.draw_debug(frame, ball, width, height)

    # ------------------------------------------------------------------ detection
    def detect_ball(self, frame):
        """Return (cx, cy, radius) of the biggest blue blob in pixels, or None. Publishes the mask."""
        # BGR -> HSV (hue = colour, saturation = how strong, value = how bright)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Read the limits every frame, so 'ros2 param set' works without restarting
        lower = np.array([self.get_parameter('h_min').value,
                          self.get_parameter('s_min').value,
                          self.get_parameter('v_min').value])
        upper = np.array([self.get_parameter('h_max').value,
                          self.get_parameter('s_max').value,
                          self.get_parameter('v_max').value])
        mask = cv2.inRange(hsv, lower, upper)   # 255 where the pixel is inside the range

        # Publish the mask (black/white image) for checking in RViz
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = self.current_header   # same time stamp and frame_id as the camera image
        self.mask_pub.publish(mask_msg)

        # Outlines of all white areas; keep only the biggest one = the ball
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) < self.min_area:
            return None   # just a speck of noise

        # Smallest circle around the contour -> centre and radius (pixels)
        (cx, cy), radius = cv2.minEnclosingCircle(biggest)
        return cx, cy, radius

    # ------------------------------------------------------------------ control
    def compute_control(self, x_n, d_n, stamp):
        """PD controller: features -> (v, omega)."""
        x_des = self.get_parameter('x_desired').value
        d_des = self.get_parameter('d_desired').value
        kp_v = self.get_parameter('kp_v').value
        kd_v = self.get_parameter('kd_v').value
        kp_w = self.get_parameter('kp_w').value
        kd_w = self.get_parameter('kd_w').value

        # Errors (desired - measured)
        e_w = x_des - x_n   # > 0: ball is LEFT of centre  -> turn left  (omega > 0)
        e_v = d_des - d_n   # > 0: ball looks too SMALL    -> drive forward (v > 0)

        # Derivatives by backward difference; dt from the image time stamps.
        # First image (or first after losing the ball): no previous error -> derivative 0.
        de_v = 0.0
        de_w = 0.0
        if self.prev_time is not None:
            dt = stamp - self.prev_time
            if dt > 0.0:   # skip if two images have the same/older stamp (avoid dividing by 0)
                de_v = (e_v - self.prev_e_v) / dt
                de_w = (e_w - self.prev_e_w) / dt

        # Remember this step for the next derivative
        self.prev_e_v = e_v
        self.prev_e_w = e_w
        self.prev_time = stamp

        # Arrived: ball centred AND at the desired size -> full stop
        if (abs(e_v) < self.get_parameter('d_tolerance').value and
                abs(e_w) < self.get_parameter('x_tolerance').value):
            return 0.0, 0.0

        # PD
        v = kp_v * e_v + kd_v * de_v
        w = kp_w * e_w + kd_w * de_w

        # Clamp omega first, then v
        w_max = self.get_parameter('max_angular_speed').value
        w = max(-w_max, min(w_max, w))

        # Each wheel's speed is v +- (track/2)*omega and must stay below MAX_WHEEL_SPEED,
        # otherwise the motor saturates and the robot can no longer steer.
        # So leave room for the turn: |v| <= MAX_WHEEL_SPEED - (track/2)*|omega|
        v_max = min(self.get_parameter('max_linear_speed').value,
                    self.MAX_WHEEL_SPEED - (self.TRACK_WIDTH / 2.0) * abs(w))
        v = max(-v_max, min(v_max, v))

        return v, w

    def handle_no_ball(self):
        """Task 8.1: stop and forget the derivative history. (Task 8.2: rotate to search here.)"""
        self.reset_derivative()
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info('no ball -> stop', throttle_duration_sec=1.0)

    def reset_derivative(self):
        """Without this, the first derivative after the ball reappears would be a huge spike."""
        self.prev_e_v = None
        self.prev_e_w = None
        self.prev_time = None

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)    # forward speed [m/s]
        cmd.angular.z = float(w)   # turning speed [rad/s], positive = counter-clockwise = left
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ debug image
    def draw_debug(self, frame, ball, width, height):
        """Draw the desired x* (vertical line), desired d* (circle) and the detected ball, then publish."""
        x_des = self.get_parameter('x_desired').value
        d_des = self.get_parameter('d_desired').value

        # Desired: yellow vertical line at x*, yellow circle with diameter d* in the image centre
        x_des_px = int(x_des * width)
        cv2.line(frame, (x_des_px, 0), (x_des_px, height), (0, 255, 255), 1)
        cv2.circle(frame, (x_des_px, height // 2), int(d_des * width / 2), (0, 255, 255), 1)

        # Detected ball: green ring + centre dot
        if ball is not None:
            cx, cy, radius = ball
            cv2.circle(frame, (int(cx), int(cy)), int(radius) + 2, (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 2, (0, 255, 0), -1)

        tracking_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        tracking_msg.header = self.current_header
        self.tracking_pub.publish(tracking_msg)

    def destroy_node(self):
        """Try to send one last stop command so the robot does not keep driving after Ctrl+C."""
        try:
            self.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoing()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
