#!/usr/bin/env python3
"""Task 6.1 - image publisher: plays the calibration photos like a camera.

  reads   share/ias0220_243483/data/images/*.png (installed by CMakeLists.txt)
  publishes /image_raw (sensor_msgs/Image) at 2 Hz, looping forever
"""

import os

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        # Find the image folder inside the installed package (no hard-coded path)
        package_dir = get_package_share_directory('ias0220_243483')
        self.image_dir = os.path.join(package_dir, 'data', 'images')

        # List the image files and sort them, so the order is always the same
        self.image_files = sorted(
            f for f in os.listdir(self.image_dir) if f.endswith('.png'))
        if not self.image_files:
            raise RuntimeError('No .png images found in ' + self.image_dir)
        self.get_logger().info(
            f'Found {len(self.image_files)} images in {self.image_dir}')

        self.index = 0             # which image to publish next
        self.bridge = CvBridge()   # converts OpenCV images <-> ROS Image messages

        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)   # 0.5 s = 2 Hz

    def timer_callback(self):
        # Load the current image from disk (OpenCV loads it as BGR)
        path = os.path.join(self.image_dir, self.image_files[self.index])
        cv_image = cv2.imread(path)

        # Convert to a ROS message and fill in the header
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)

        # Go to the next image, and back to the first one after the last
        self.index = (self.index + 1) % len(self.image_files)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
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
