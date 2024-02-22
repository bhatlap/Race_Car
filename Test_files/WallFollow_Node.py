#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan as lscan
from ackermann_msgs.msg import AckermannDriveStamped as acker
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry as odom
from tf.transformations import euler_from_quaternion


class WallFollow:

    def __init__(self):
        # PID PARAMS
        self.kp = 1
        self.kd = 0.05
        self.ki = 1
        self.errors = [0.0, 0.0, 0.0]
        self.dt = 0.002

        # SUBSCRIBERS AND PUBLISHERS
        self.scan_sub = rospy.Subscriber('/scan', lscan, self.scan_callback)  # Subscribe to LIDAR
        self.odom_sub = rospy.Subscriber('/odom', odom, self.odom_callback)

        # MISC VARIABLES
        self.d_r = 0.0
        self.d_l = 0.0
        self.ref_d = 0.5
        self.scan_msg = None
        self.heading = None

    def get_range(self, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:

            angle: any angle such that angle_min < angle < angle_max of the LiDAR

        Returns:
            r: range measurement in meters at the given angle
        """
        theta0 = self.scan_msg.angle_min
        del_theta = self.scan_msg.angle_increment
        idx = int((angle - theta0)/del_theta)
        r = self.scan_msg.ranges[idx]

        return r

    def get_error(self, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counterclockwise in the Levine loop).
        You potentially will need to use get_range()

        Args:
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        theta1 = -np.pi / 2
        theta2 = np.pi / 2

        theta = - 160 * np.pi/180   # Known angle for measuring alpha value

        a = self.get_range(angle=theta)
        b = self.get_range(angle=theta1)
        c = self.get_range(angle=theta2)

        alpha = np.arctan(a * np.cos(abs(theta)) - b) / (a * np.sin(abs(theta)))
        # alpha = self.heading
        self.d_r = a * np.cos(abs(alpha))
        self.d_l = c * np.cos(abs(alpha))

        error = self.d_l - dist

        return error

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            None
        """
        drive_pub = rospy.Publisher('/drive', acker, queue_size=10)  # Publish to drive topic

        d_error = (self.errors[2] - 2*self.errors[1] + self.errors[0])/(2*self.dt)
        I_error = (self.dt/2) * (self.errors[2] + 2*self.errors[1] + self.errors[0])

        angle = self.kp*error + self.kd*d_error + self.ki*I_error

        if 0 <= abs(angle) < 0.175:
            velocity = 1.5
        elif 0.175 <= abs(angle) < 0.350:
            velocity = 1.0
        else:
            velocity = 0.5

        drive_msg = acker()

        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.scan_msg = msg  # Store latest message

        # Calculate error
        error = self.get_error(self.ref_d)

        print(error)
        # Update error buffer
        self.errors[0] = self.errors[1]
        self.errors[1] = self.errors[2]
        self.errors[2] = error

        # Drive the car using PID controller
        self.pid_control(error)

    def odom_callback(self, msg):

        orientation = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w)
                                                 )
        self.heading = orientation[2]


def main():
    rospy.init_node('WallFollow',anonymous=True)
    wf = WallFollow()
    rospy.spin()


if __name__ == '__main__':
    main()