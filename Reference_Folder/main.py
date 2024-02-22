#!/usr/bin/env python3

import numpy as np

# ROS imports
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from nav_msgs.msg import Odometry, Path
import rospkg

# Helper scripts
from Helper_scripts.Models import Kinematic_Single_Track_Model,KSTModel
from Helper_scripts.simulate import simulate_states
from tf.transformations import euler_from_quaternion


global x, x_ref


def refCallback(trajectory):

    global x, x_ref
    """
    Function to process the reference trajectory for MPC implementation
    """
    x_ref = []

    for i,pose in enumerate(trajectory.poses):
        _,_,psi = euler_from_quaternion([pose.pose.orientation.x,
                                         pose.pose.orientation.y,
                                         pose.pose.orientation.z,
                                         pose.pose.orientation.w])
        x_ref.append([pose.pose.posiiton.x,pose.pose.posiiton.y,psi])
        x_ref = np.array(x_ref)


def odomCallback(odometry):

    global x, x_ref
    """
    Function to process the odometry data from the simulation    
    """

    odom_data = odometry


def main():

    global x, x_ref

    rospack = rospkg.RosPack()
    track = 'Silverstone'
    pkg_path = rospack.get_path('f1tenth_simulator')
    file_path = pkg_path + f'/scripts/Additional_maps/{track}/{track}_centerline.csv'

    reftrajSub = rospy.Subscriber('/ref_trajectory', Path, refCallback)
    odomSub = rospy.Subscriber('/odom', Odometry, odomCallback)






if __name__ == '__main__':
    main()

    # u = np.array([[0, 0]])
    # for i in range(1, 100):
    #     vel = 1
    #     steer_ang = np.sin(np.pi * i / 180)
    #     u_new = np.array([[steer_ang, vel]])
    #     u = np.vstack((u, u_new))
    #
    # simulate_states(car, x_0, u)
