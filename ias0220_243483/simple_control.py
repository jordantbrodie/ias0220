#!/usr/bin/env python3

"""
Solution to home assignment 7 (Robot Control). Node to take a set of
waypoints and to drive a differential drive robot through those waypoints
using a simple PD controller and provided odometry data.

@author: Yuya Hamamatsu
@date: 04.11.23
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message.
"""

import math
import rclpy
import time
from rclpy.node import Node
from rclpy.time import Duration, Time
import numpy as np
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class PDController(Node):
    def __init__(self):
        # Your code here
        super().__init__('controller')
        
        # Wait for run other nodes
        time.sleep(5)

        # variables for error change rate calculation
        self.start_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        self.sub_odom = self.create_subscription(
            Odometry, '/diff_cont/odom', self.onOdom, 10)

        self.vel_cmd_pub = self.create_publisher(
            Twist, '/diff_cont/cmd_vel', 10)
        self.pub_viz = self.create_publisher(Marker, "waypoints", 10)

        self.marker_frame = "odom"

        self.vel_cmd_msg = Twist()
        self.vel_cmd = np.array([0.0, 0.0])

        self.pos = np.array([0.0, 0.0])
        self.pos_diff = np.array([0.0, 0.0])
        self.theta = 0.0
        self.th_diff = 0.0

        self.error = np.array([0.0, 0.0])

        self.error_change_rate = np.array([0.0, 0.0])

        # Load params from parameter server
        self.waypoints = []
        self.waypoints_x = []
        self.waypoints_y = []
        read_waypoints_x = self.declare_parameter('waypoints_x', [0.0]).value
        read_waypoints_y = self.declare_parameter('waypoints_y', [0.0]).value
        self.Kp = np.array(self.declare_parameter('Kp', [0.0, 0.0]).value)
        self.Kd = np.array(self.declare_parameter('Kd', [0.0, 0.0]).value)
        # Saturation limits [linear m/s, angular rad/s] so big errors don't give crazy speeds
        self.max_vel = np.array(self.declare_parameter('max_vel', [1.0, 2.0]).value)
        self.dt = 0.0
        self.first_odom = True

        # Identify the waypoints parameter is correcly load or not
        if (read_waypoints_x == [0.0]) and (read_waypoints_y == [0.0]):
            self.get_logger().error("!!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!!")
            self.get_logger().error("Parameters not loaded correctly")
            self.get_logger().error("Please check your launch file to load yaml file correctly")
            self.get_logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        if (read_waypoints_x) and (read_waypoints_y):
            for i in range(len(read_waypoints_x)):
                self.waypoints.append([read_waypoints_x[i], read_waypoints_y[i]])
        self.waypoints = np.array(self.waypoints)

        self.distance_margin = self.declare_parameter(
            'distance_margin', 0.05).value
        self.target_angle = self.wrapAngle(self.declare_parameter(
            'target_angle', 0.0).value)

        # Check Params
        self.get_logger().info(f'Waypoints: \n {self.waypoints}')
        self.get_logger().info(f'Kp: {self.Kp}')
        self.get_logger().info(f'Kd: {self.Kd}')
        self.get_logger().info(f'Start Time: {self.start_time}')
        

    def wrapAngle(self, angle):
        """
        Helper function that returns angle wrapped between +- Pi.
        Hint: Pass your error in heading [rad] into this function, and it
        returns the shorter angle. This prevents your robot from turning
        along the wider angle and makes it turn along the smaller angle (but
        in opposite direction) instead.
        @param: self
        @param: angle - angle to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """
        
        # atan2(sin, cos) maps any angle to the same direction in [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))
        
    def control(self):
        """
        Takes the errors and calculates velocities from it, according to
        PD control algorithm.
        @param: self (errors got using "calculateError" function)
        @result: sets the values in self.vel_cmd
        """

        # Mission finished: stand still
        if self.waypoints.size == 0:
            self.vel_cmd = np.array([0.0, 0.0])
            return

        # PD law, element-wise: [v, w] = Kp*[e_pos, e_th] + Kd*[de_pos/dt, de_th/dt]
        self.vel_cmd = self.Kp * self.error + self.Kd * self.error_change_rate

        # Turn first, then drive: scale forward speed by cos(heading error).
        # Facing the goal -> full speed; 90 deg off or more -> no forward motion.
        self.vel_cmd[0] *= max(0.0, math.cos(self.error[1]))

        # Saturate to the limits (no reversing)
        self.vel_cmd[0] = float(np.clip(self.vel_cmd[0], 0.0, self.max_vel[0]))
        self.vel_cmd[1] = float(np.clip(self.vel_cmd[1], -self.max_vel[1], self.max_vel[1]))

    def publishWaypoints(self):
        """
        Publishes the list of waypoints, so RViz can see them.
        @param: self
        @result: publish message
        """
        marker = Marker()
        marker.header.frame_id = self.marker_frame

        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD

        marker.scale.x = self.distance_margin
        marker.scale.y = self.distance_margin
        marker.scale.z = self.distance_margin
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.w = 1.0

        marker.points = [Point(x=waypoint[0], y=waypoint[1])
                         for waypoint in self.waypoints]

        self.pub_viz.publish(marker)

    def calculateError(self):
        """
        Calculate lateral error to first waypoint and heading error between
        line to waypoint and robot heading.
        @param: self
        @result: updates self.error, self.error_change_rate, self.th_diff and
                 self.pos_diff
        """
        if self.waypoints.size == 0:
            return

        goal = self.waypoints[0]
        # Vector from robot to goal
        self.pos_diff = goal - self.pos
        # Distance error and heading error (shortest way round)
        e_pos = float(np.hypot(self.pos_diff[0], self.pos_diff[1]))
        th_goal = math.atan2(self.pos_diff[1], self.pos_diff[0])
        self.th_diff = self.wrapAngle(th_goal - self.theta)
        new_error = np.array([e_pos, self.th_diff])

        # Derivative: (e - e_prev) / dt, dt from consecutive odom stamps.
        # Skip on the first message / bad dt, and use wrapAngle on the heading
        # difference so a +pi -> -pi flip is not seen as a huge jump.
        if self.first_odom or self.dt <= 0.0 or self.dt > 1.0:
            self.error_change_rate = np.array([0.0, 0.0])
        else:
            d_pos = new_error[0] - self.error[0]
            d_th = self.wrapAngle(new_error[1] - self.error[1])
            self.error_change_rate = np.array([d_pos, d_th]) / self.dt
        self.first_odom = False
        self.error = new_error

    def isWaypointReached(self):
        """
        check if waypoint is reached based on user define threshold
        @param: self
        @result: if waypoint is reached that waypoint is popped from
                 waypoint list and True is returned,
                 otherwise False is returned.
        """

        if self.waypoints.size != 0:
            if (self.error[0] < self.distance_margin):
                self.waypoints = np.delete(self.waypoints, 0, axis=0)
                return True

        return False

    def onOdom(self, odom_msg):
        """
        Handles incoming odometry updates (callback function).
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """

        self.pos[0] = odom_msg.pose.pose.position.x
        self.pos[1] = odom_msg.pose.pose.position.y

        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.theta = euler_from_quaternion(orientation_list)[2]
        
        now_odom_time = rclpy.time.Time.from_msg(odom_msg.header.stamp)
        
        dt_tmp = (now_odom_time - self.last_odom_time).to_msg()
        self.dt = float(dt_tmp.sec + dt_tmp.nanosec/1e9)
        self.last_odom_time = now_odom_time
        
        # Calculate error between current pose and next waypoint position
        self.calculateError()

        # Check reaching waypoint or not
        if self.isWaypointReached():
            self.get_logger().info(
                "Reached waypoint!\nFuture waypoint list: "
                + str(self.waypoints))
            self.first_odom = True  # new goal: don't differentiate across the jump
            self.calculateError()  # Update error with new target waypoint

        # Calculate velocity command using PID control
        self.control()

        # publish velocity commands
        self.vel_cmd_msg.linear.x = self.vel_cmd[0]
        self.vel_cmd_msg.angular.z = self.vel_cmd[1]
        self.vel_cmd_pub.publish(self.vel_cmd_msg)

        # Publish waypoints visualization
        self.publishWaypoints()
        


def main(args=None):
    rclpy.init(args=args)
    controller = PDController()
    rclpy.spin(controller)
    
if __name__ == "__main__":
    main()
