#!/usr/bin/env python3
"""Task 6.2 - object recognition: finds the red ball by colour.

  subscribes /camera1/image_raw (sensor_msgs/Image, from the dr_mujoco camera)
  publishes  /ball_mask         (mono8 Image: white = pixel has the ball's colour)
             /ball_tracking     (bgr8 Image: camera image with the detected ball circled)

Red sits at both ends of OpenCV's hue scale (0..180), so two HSV ranges are used:
  low red  : hue from 0             to red_low_h_max
  high red : hue from red_high_h_min to 180
Both share the same saturation and value limits. All limits are ROS parameters,
so they can be tuned live with:  ros2 param set /object_recognition <name> <value>
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObjectRecognition(Node):

    def __init__(self):
        super().__init__('object_recognition')

        # HSV limits, tuned on 200 recorded frames:
        #   ball hue 0 (edges 1-5 and 170-179), barrel hue 19-20 (edges down to 11)
        #   -> low-red limit 8 sits in the gap, so the orange barrel is never detected.
        #   Ball and barrel are equally saturated (S > 170), so hue is what separates them.
        #   The ball's shadowed underside is dark (V down to ~10), hence v_min 20.
        self.declare_parameter('red_low_h_max', 8)      # low red:  hue 0 .. this
        self.declare_parameter('red_high_h_min', 170)   # high red: this .. 180
        self.declare_parameter('s_min', 150)            # strong colour only (not grey/white)
        self.declare_parameter('v_min', 20)             # not (almost) black
        self.declare_parameter('v_max', 255)

        # Contours smaller than this (in pixels^2) are treated as noise.
        # Smallest real ball piece seen: area 8 (ball peeking out behind an obstacle).
        self.min_area = 5.0

        # Bonus: every detected ball centre, used to draw the ball's path
        self.path = []

        self.bridge = CvBridge()   # converts ROS Image <-> OpenCV image (numpy array)

        self.image_sub = self.create_subscription(
            Image, '/camera1/image_raw', self.image_callback, 10)
        self.mask_pub = self.create_publisher(Image, '/ball_mask', 10)
        self.tracking_pub = self.create_publisher(Image, '/ball_tracking', 10)

        self.get_logger().info('object_recognition started, waiting for /camera1/image_raw')

    def image_callback(self, msg):
        # 1. ROS Image -> OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. BGR -> HSV (hue = colour, saturation = how strong, value = how bright)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 3. Read the current limits (so 'ros2 param set' works without restarting)
        low_h_max = self.get_parameter('red_low_h_max').value
        high_h_min = self.get_parameter('red_high_h_min').value
        s_min = self.get_parameter('s_min').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value

        # 4. Two masks, one for each end of the hue scale, then join them
        lower1 = np.array([0, s_min, v_min])
        upper1 = np.array([low_h_max, 255, v_max])
        lower2 = np.array([high_h_min, s_min, v_min])
        upper2 = np.array([180, 255, v_max])
        mask1 = cv2.inRange(hsv, lower1, upper1)   # 255 where the pixel is inside the range
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 5. Publish the mask (black/white image) for tuning in RViz
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header   # same time stamp and frame_id as the camera image
        self.mask_pub.publish(mask_msg)

        # 6. Find the outlines (contours) of all white areas in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 7. Keep only the biggest one = the ball (ignore it if it is just a speck)
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest) >= self.min_area:
                # 8. Smallest circle around the contour -> centre and radius, draw it
                (x, y), radius = cv2.minEnclosingCircle(biggest)
                centre = (int(x), int(y))
                self.path.append(centre)   # bonus: remember where the ball was
                cv2.circle(frame, centre, int(radius) + 2, (0, 255, 0), 2)   # green ring
                cv2.circle(frame, centre, 2, (0, 255, 0), -1)                # centre dot

        # Bonus: draw a line through all centres found so far (on every frame).
        # The ball moves left/right, so this becomes a horizontal line; its length
        # shows how much of the movement we detect.
        if len(self.path) >= 2:
            points = np.array(self.path, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [points], isClosed=False, color=(255, 255, 0), thickness=1)

        # 9. Publish the processed image EVERY frame (also when no ball is found)
        tracking_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        tracking_msg.header = msg.header
        self.tracking_pub.publish(tracking_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognition()
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
