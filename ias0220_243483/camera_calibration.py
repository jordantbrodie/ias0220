#!/usr/bin/env python3
"""Task 6.1 - camera calibration node.

  subscribes /image_raw       (sensor_msgs/Image)
  publishes  /image_processed (sensor_msgs/Image)      chessboard corners drawn on the image
             /camera_info     (sensor_msgs/CameraInfo) after the calibration is done

All work happens in the image callback, in three states:
  COLLECTING -> CALIBRATING -> PUBLISHING
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

# Chessboard from exercise_1.pdf
PATTERN_SIZE = (7, 6)   # inner corners: 7 per row, 6 per column
SQUARE_SIZE = 0.108     # m
IMAGES_NEEDED = 37      # images with corners found before we calibrate


class CameraCalibration(Node):

    def __init__(self):
        super().__init__('camera_calibration')

        self.bridge = CvBridge()
        self.state = 'COLLECTING'

        # 3D corner positions on the flat board (z = 0): (0,0,0), (0.108,0,0), ...
        # They are the same for every image, because the board itself never changes.
        self.objp = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
        self.objp[:, :2] = (np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
                            * SQUARE_SIZE)

        self.objpoints = []   # 3D points in the board frame, one array per image
        self.imgpoints = []   # 2D corner pixels in the image, one array per image

        # Stop refining corners after 30 iterations or when they move < 0.001 px
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.processed_pub = self.create_publisher(Image, '/image_processed', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Calibration results, filled in the CALIBRATING state
        self.mtx = None    # 3x3 camera matrix K
        self.dist = None   # distortion coefficients (k1, k2, p1, p2, k3)

        self.get_logger().info(f'Collecting {IMAGES_NEEDED} images with chessboard corners')

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.state == 'COLLECTING':
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, None)

            if found:
                # Refine the corners to sub-pixel accuracy and store them
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                cv2.drawChessboardCorners(image, PATTERN_SIZE, corners2, found)
                self.get_logger().info(
                    f'Corners found: {len(self.imgpoints)}/{IMAGES_NEEDED}')
            else:
                self.get_logger().warn('No corners found in this image, skipping it')

            # Publish the image (with corners drawn if found), same header as the input
            processed_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)

            if len(self.imgpoints) >= IMAGES_NEEDED:
                self.state = 'CALIBRATING'
                self.get_logger().info('Enough images collected, calibrating')

        elif self.state == 'CALIBRATING':
            # Image size as (width, height), same as gray.shape[::-1] in exercise_1.pdf
            image_size = (msg.width, msg.height)
            rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, image_size, None, None)

            # Re-projection error (exercise_1.pdf): project the known 3D corners into
            # each image with the calibration result and compare with the detected corners
            total_error = 0.0
            for i in range(len(self.objpoints)):
                imgpoints_known, _ = cv2.projectPoints(
                    self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(self.imgpoints[i], imgpoints_known, cv2.NORM_L2)
                total_error += error / len(imgpoints_known)
            mean_error = total_error / len(self.objpoints)

            self.get_logger().info(f'Camera matrix K:\n{mtx}')
            self.get_logger().info(f'Distortion coefficients: {dist.flatten()}')
            self.get_logger().info(f'Mean re-projection error: {mean_error:.5f}')
            self.get_logger().info(f'calibrateCamera RMS error: {rms:.5f} px')

            self.mtx = mtx
            self.dist = dist
            self.state = 'PUBLISHING'
            self.get_logger().info('Calibration done, publishing /camera_info')

        elif self.state == 'PUBLISHING':
            info = CameraInfo()
            info.header = msg.header   # same stamp as the image, so image_proc can match them
            info.height = msg.height
            info.width = msg.width
            info.distortion_model = 'plumb_bob'
            info.d = self.dist.flatten().tolist()
            info.k = self.mtx.flatten().tolist()
            info.r = np.eye(3).flatten().tolist()   # no rotation (single camera, not stereo)
            # P = [K | 0]: camera matrix with a zero column added on the right (3x4)
            info.p = np.hstack((self.mtx, np.zeros((3, 1)))).flatten().tolist()
            self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
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
