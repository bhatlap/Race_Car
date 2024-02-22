#!/usr/bin/env python3

import numpy as np

# ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped as acker


def generate_drive_msg(speed, heading):
    """
    Function to drive car
    """

    drive_msg = acker()

    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.header.frame_id = "laser"
    drive_msg.drive.steering_angle = heading
    drive_msg.drive.speed = speed

    return drive_msg
