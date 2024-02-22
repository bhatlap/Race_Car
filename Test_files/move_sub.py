#!/usr/bin/env python3
#
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class odom:
    def __init__(self):
        self.pose = np.zeros([3, 1])
        self.Odomsub = rospy.Subscriber('/odom', Odometry, self.odomCallback)

    def odomCallback(self, odom_data):

        _,_,psi = euler_from_quaternion([odom_data.pose.pose.orientation.x,
                                         odom_data.pose.pose.orientation.y,
                                         odom_data.pose.pose.orientation.z,
                                         odom_data.pose.pose.orientation.w])

        self.pose = np.array([odom_data.pose.pose.position.x,
                              odom_data.pose.pose.position.y,
                              psi
                             ])


# def main():
#     rospy.init_node('Move', anonymous=True)
#     car_odometry = odom()
#     rate = rospy.Rate(50)
#     while not rospy.is_shutdown():
#         position = car_odometry.pose[:2]
#
#         x_current = position[:1]
#         y_current = position[1:]
#         print(x_current[:20])
#
#
# if __name__ == '__main__':
#     main()
