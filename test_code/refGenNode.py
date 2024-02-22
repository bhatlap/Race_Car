#!/usr/bin/env python3
import numpy as np

import rospy
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg


def visualize_center():
    rospack = rospkg.RosPack()
    track = 'Silverstone'
    pkg_path = rospack.get_path('f1tenth_simulator')
    file_path = pkg_path + f'/scripts/Additional_maps/{track}/{track}_centerline.csv'
    df = pd.read_csv(file_path)

    x_vals = df.iloc[:, 0].values
    y_vals = df.iloc[:, 1].values

    x_vals = np.append(x_vals, x_vals[0])
    y_vals = np.append(y_vals, y_vals[0])

    rospy.init_node('refGenNode', anonymous=True)
    markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(1000)

    lineMsg = Marker()
    i = 0
    while not rospy.is_shutdown():
        lineMsg.header.frame_id = 'map'
        lineMsg.header.stamp = rospy.Time.now()
        lineMsg.id = 0
        lineMsg.type = Marker.LINE_STRIP
        lineMsg.action = Marker.ADD
        lineMsg.pose.orientation.w = 1.0
        lineMsg.scale.x = 0.05
        lineMsg.ns = 'Center_line'

        lineMsg.color.r = 1
        lineMsg.color.a = 1
        lineMsg.lifetime = rospy.Duration(0)

        if i < len(x_vals)-1:
            point1 = Point()
            point2 = Point()
            point1.x = x_vals[i]
            point1.y = y_vals[i]
            point2.x = x_vals[i+1]
            point2.y = y_vals[i+1]
            lineMsg.points.append(point1)
            lineMsg.points.append(point2)
            i = i+1
        markerPub.publish(lineMsg)
        rate.sleep()


if __name__ == '__main__':
    try:
        visualize_center()
    except rospy.ROSInterruptException:
        pass


# Point Example
  # Point publishing
    # i = 0
    # while not rospy.is_shutdown():
    #     if i <= len(x_vals):
    #         # Create point message
    #         pointMsg = Marker()
    #         pointMsg.header.frame_id = 'map'
    #         pointMsg.header.stamp = rospy.Time.now()
    #         pointMsg.id = 0
    #         pointMsg.ns = 'Points'
    #         pointMsg.type = Marker.POINTS
    #         pointMsg.action = Marker.ADD
    #         pointMsg.pose.orientation.w = 1.0
    #         pointMsg.scale.x = 0.5
    #         pointMsg.scale.y = 0.5
    #         pointMsg.color.r = 1.0
    #         pointMsg.color.a = 1.0
    #
    #         # Create actual point
    #         point = Point()
    #         point.x = x_vals[i]
    #         point.y = y_vals[i]
    #         point.z = 0
    #         print(point.x)
    #         # Add point to point message
    #         pointMsg.points.append(point)
    #         i = i+1
    #         markerPub.publish(pointMsg)
    #         rate.sleep()
    #     else:
    #         print(i)

    # Line publishing