#!/usr/bin/env python3

import pandas as pd
import numpy as np
import cv2
import matplotlib.pyplot as plt

# ROS Imports
import rospy
import rospkg
import tf2_ros


def parseTrack(img_file_path):
    """
    Function to extract center line and track bounds using the Euclidean distance transform.
    :Inputs:
        img_file_path : Path to image of track
    :results:
        track_bounds : X and Y coordinates of track bounds as a numpy array
        center_line: X and Y coordinates of center line as a numpy array
    """
    track_map = cv2.imread(img_file_path, cv2.IMREAD_GRAYSCALE)

    _, center_line_bin_map = cv2.threshold(track_map, 250,255, cv2.THRESH_BINARY)
    # cv2.imwrite('Center_Line_binary_map.png', center_line_bin_map)

    center_dist_transform = cv2.distanceTransform(center_line_bin_map, cv2.DIST_L2, 3)
    # cv2.imwrite('Center_Line_dist_transform.png', center_dist_transform)

    norm_center_dist_tf = cv2.normalize(center_dist_transform, None, 0, 255, cv2.NORM_MINMAX)
    # cv2.imwrite('Center_Line_norm_dist_tf.png', norm_center_dist_tf)

    _, center_line_thresholded = cv2.threshold(norm_center_dist_tf, 166, 255, cv2.THRESH_BINARY)
    # cv2.imwrite('Center_Line_thresholded.png', center_line_thresholded)

    center_line_thinned_img = cv2.ximgproc.thinning(np.uint8(center_line_thresholded))
    # cv2.imwrite('Center_Line_thinned_img.png', center_line_thinned_img)

    center_line = np.column_stack(np.where(center_line_thinned_img > 0))

    kernel1 = np.ones((21, 21), np.uint8)
    kernel2 = np.ones((13, 13), np.uint8)

    complete_track = cv2.dilate(center_line_thinned_img, kernel1, iterations=1)
    # cv2.imwrite('complete_track_binary_img.png', complete_track)

    dilated_center_line = cv2.dilate(center_line_thinned_img, kernel2, iterations=1)
    # cv2.imwrite('dilated_center_line_img.png', dilated_center_line)

    track_bounds_img = cv2.subtract(complete_track, dilated_center_line )
    # cv2.imwrite('track_bounds_img.png', track_bounds_img)

    track_bounds_thinned_img = cv2.ximgproc.thinning(np.uint8(track_bounds_img))
    # cv2.imwrite('track_bounds_thinned_img.png', track_bounds_thinned_img)

    track_bounds = np.column_stack(np.where(track_bounds_thinned_img > 0))

    plt.plot(center_line[:, 0], center_line[:, 1], '.')
    plt.plot(track_bounds[:, 0], track_bounds[:, 1], '.')
    plt.show()

    return track_bounds, center_line


def get_bounds(center_line, dist):

    dx_c = np.diff(center_line[:, 0])
    dy_c = np.diff(center_line[:, 1])

    d = dist
    abs_val = np.sqrt(dx_c**2 + dy_c**2)
    # Calculate unit normal vector
    unit_normal = np.array([-dy_c/abs_val, dx_c/abs_val]).T

    x_left = center_line[:-1, 0] + d*unit_normal[:, 0]
    y_left = center_line[:-1, 1] + d*unit_normal[:, 1]

    x_right = center_line[:-1, 0] - d*unit_normal[:, 0]
    y_right = center_line[:-1, 1] - d*unit_normal[:, 1]

    left_bounds = np.column_stack([x_left,y_left])
    right_bounds = np.column_stack([x_right,y_right])

    return left_bounds, right_bounds


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('f1tenth_simulator')
    map_path = pkg_path + f'/maps/levine_blocked.pgm'
    center_line_path = pkg_path + f'/scripts/Additional_maps/Silverstone/Silverstone_centerline.csv'

    df = pd.read_csv(center_line_path)
    center_line = df.iloc[0::2, :2].values
    get_bounds(center_line)
    # track_bounds, center_line = parseTrack(file_path)
    # dist_to_boundary(track_bounds,center_line)

    # plt.plot(track_bounds[:, 0], track_bounds[:, 1], '.')
    # plt.show()