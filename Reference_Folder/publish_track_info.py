#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import numpy as np


def read_csv(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data


def extract_track_data(data):
    centerline = data[:, 0:2]  # Assuming the CSV format has X, Y coordinates for centerline in columns 2 and 3
    distance_from_center = data[:, 3]  # Assuming the CSV format has distance from center in column 5
    return centerline, distance_from_center


def generate_borders(centerline, distance_from_center):
    angles = np.arctan2(np.gradient(centerline[:, 1]), np.gradient(centerline[:, 0]))
    perpendicular_angles = angles + np.pi / 2.0

    left_border_x = centerline[:, 0] + distance_from_center * np.cos(perpendicular_angles)
    left_border_y = centerline[:, 1] + distance_from_center * np.sin(perpendicular_angles)

    right_border_x = centerline[:, 0] - distance_from_center * np.cos(perpendicular_angles)
    right_border_y = centerline[:, 1] - distance_from_center * np.sin(perpendicular_angles)

    left_border = np.column_stack((left_border_x, left_border_y))
    right_border = np.column_stack((right_border_x, right_border_y))

    return left_border, right_border


def publish_points_on_topic(points, topic_name):
    rospy.init_node('track_publisher', anonymous=True)
    point_pub = rospy.Publisher(topic_name, PointStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        for point in points:
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "map"
            point_msg.point.x = point[0]
            point_msg.point.y = point[1]
            point_pub.publish(point_msg)
            rate.sleep()


if __name__ == "__main__":
    # Specify the path to your CSV file
    csv_file_path = "/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/Silverstone/Silverstone_centerline.csv"

    # Read CSV file
    track_data = read_csv(csv_file_path)

    # Extract centerline and distance from center
    centerline_points, distance_from_center = extract_track_data(track_data)

    # Generate left and right border points
    left_border_points, right_border_points = generate_borders(centerline_points, distance_from_center)

    # Publish centerline, left border, and right border points on separate ROS topics
    publish_points_on_topic(centerline_points, '/centerline_points')
    publish_points_on_topic(left_border_points, '/left_border_points')
    publish_points_on_topic(right_border_points, '/right_border_points')
