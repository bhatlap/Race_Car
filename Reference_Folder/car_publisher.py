#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def move_car():
    rospy.init_node('move_car', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1)  # Set a publishing rate of 1 Hz

    while not rospy.is_shutdown():
        # Move the car forward at a constant linear velocity
        twist_msg = Twist()
        twist_msg.linear.x = 1.0  # Set desired linear velocity
        twist_msg.angular.z = 2.0  # Set desired angular velocity (zero for straight motion)
        pub.publish(twist_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        move_car()
    except rospy.ROSInterruptException:
        pass
