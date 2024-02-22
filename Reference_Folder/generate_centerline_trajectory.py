#!/usr/bin/env python3

import pandas as pd
import numpy as np

# ROS Imports
import rospkg
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from generate_vizMarkerMsg import generate_vizMarkerMsg
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


def reference_traj_generator(file_path, t_samp=1.0):
    """
    Function to generate reference trajectory, along the center line of the racetrack. This function uses linear
    interpolation to extract information about reference trajectory if sampling time is less than one second.
    Default sampling time is 1 second

    :Inputs:
        file_path : Path to .csv file containing the x and y coordinates of the track center line
        t_samp : Sampling time of MPC algorithm in seconds

    :return:
        traj : trajectory message of type nav_msgs/Path which contains references poses for every t_samp seconds.
    """
    traj = Path()
    traj.header = Header()
    traj.header.frame_id = 'map'
    traj.header.stamp = rospy.Time(0)

    df = pd.read_csv(file_path)
    points = df.iloc[0::2, :2].values
    points = np.vstack([points, points[0, :]])

    if t_samp < 1:
        # Generate timestamps assuming each coordinate is covered in 1 second
        timestamps = np.arange(0, len(points), 1)

        # Create new timestamps for interpolation with actual sampling time
        time_interp = np.arange(timestamps[0], timestamps[-1], t_samp)

        # Interpolating position from previous sample
        x_ref = np.interp(time_interp, timestamps, points[:, 0])
        y_ref = np.interp(time_interp, timestamps, points[:, 1])
        ref_pts = np.column_stack((x_ref, y_ref))

        for i, point in enumerate(ref_pts):
            if i < len(ref_pts) - 1:
                # Calculate the orientation for each reference point
                dx = x_ref[i + 1] - x_ref[i]
                dy = y_ref[i + 1] - y_ref[i]
                psiref = np.arctan2(dy, dx)
                q_ref = quaternion_from_euler(ai=0, aj=0, ak=psiref)

            else:
                # Orientation of last point
                dx_e = x_ref[0] - x_ref[-1]
                dy_e = y_ref[0] - y_ref[-1]
                psiref_e = np.arctan2(dy_e, dx_e)
                q_ref = quaternion_from_euler(ai=0, aj=0, ak=psiref_e)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = traj.header.frame_id
            pose_stamped.pose.position = Point(point[0], point[1], 0.0)
            pose_stamped.pose.orientation.w = q_ref[3]
            pose_stamped.pose.orientation.x = q_ref[0]
            pose_stamped.pose.orientation.y = q_ref[1]
            pose_stamped.pose.orientation.z = q_ref[2]
            pose_stamped.header.stamp = traj.header.stamp + i * rospy.Duration(secs=0, nsecs=int(t_samp*1e9))
            traj.poses.append(pose_stamped)
    else:
        for i, point in enumerate(points):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(point[0], point[1], 0.0)
            pose_stamped.header.stamp = traj.header.stamp + i * rospy.Duration(int(t_samp))
            traj.poses.append(pose_stamped)

    return traj


def main():
    rospy.init_node('trajectory_generator',anonymous=True)
    trajPub = rospy.Publisher('/ref_trajectory', Path, queue_size=10)

    rospack = rospkg.RosPack()
    track = 'Silverstone'
    pkg_path = rospack.get_path('f1tenth_simulator')
    file_path = pkg_path + f'/scripts/Additional_maps/{track}/{track}_centerline.csv'

    trajMsg = reference_traj_generator(file_path=file_path, t_samp=0.02)

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        trajPub.publish(trajMsg)
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")
