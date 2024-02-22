#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def car_subscriber(data):
    # Process received data (you can visualize it in RViz)
    rospy.loginfo("Received x: %.2f, Received z: %.2f",
                  data.linear.x, data.angular.z)

def main():
    rospy.init_node('car_subscriber', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, car_subscriber)
    rospy.spin()

if __name__ == '__main__':
    main()
