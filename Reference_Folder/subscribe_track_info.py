#! usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import numpy as np
def subscriber_to_center(data):
    x_value = data.point.x
    y_value = data.point.y
    result = np.vstack((x_value,y_value)).T
    print(result)
    #rospy.loginfo("x_cordinate_center: %.2f, y_cordinate_center: %2f",data.point.x,data.point.y)


def subscriber_to_left(data):
    rospy.loginfo("x_cordinate_left: %.2f, y_cordinate_left: %2f",data.point.x,data.point.y)

def subscriber_to_right(data):
    rospy.loginfo("x_cordinate_right: %.2f, y_cordinate_right: %2f",data.point.x,data.point.y)

def main():

    rospy.init_node('track_subscriber',anonymous=True)
    rospy.Subscriber('/centerline_points',PointStamped,subscriber_to_center)
    rospy.Subscriber('/left_border_points', PointStamped, subscriber_to_left)
    rospy.Subscriber('/right_border_points', PointStamped, subscriber_to_right)
    rospy.spin()

if __name__ == '__main__':
    main()