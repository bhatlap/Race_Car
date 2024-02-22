#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import casadi as ca
import matplotlib.pyplot as plt
import rospy
import numpy as np
import pandas as pd
from move_sub import *



file_path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/Silverstone/Silverstone_centerline.csv'

df = pd.read_csv(file_path)
center_line = df.iloc[:, :2].values

def optimizer(x_current,y_current):

    N = 200  # number of control intervals

    opti = ca.Opti()  # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(3, N + 1)  # state trajectory
    x_pos = X[0, :]
    y_pos = X[1, :]
    theta = X[2,:]
    U = opti.variable(2, N)
    velocity = U[0,:]# control trajectory (throttle)
    steering = U[1,:]# control trajectory (steering)
    T = opti.variable()  # final time


    # ---- objective          ---------
    opti.minimize(T)  # race in minimal time

    # ---- dynamic constraints --------
    def f(x, u):
        return ca.vertcat(u[0]*np.cos(x[2]), u[0]*np.sin(x[2]), (u[0]*np.tan(u[1]))/0.3302)

    print(X.shape)
    print(U.shape)
    dt = T / N  # length of a control interval


    for k in range(N):  # loop over control intervals

        # Runge-Kutta 4 integration
        k1 = f(X[:, k], U[:, k])
        k2 = f(X[:, k] + dt / 2 * k1, U[:, k])
        k3 = f(X[:, k] + dt / 2 * k2, U[:, k])
        k4 = f(X[:, k] + dt * k3, U[:, k])
        x_next = X[:, k] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(X[:, k + 1] == x_next)  # close the gaps


    # ---- path constraints -----------
    #limit = lambda p: 1 - ca.sin(2 * np.pi * p) / 2
    #opti.subject_to(speed <= limit(pos))  # track speed limit
    opti.subject_to(velocity <= 0.3)  # control is limited
    opti.subject_to(velocity >= 0)  # control is limited
    opti.subject_to(steering <= np.pi)  # control is limited
    opti.subject_to(steering >= -np.pi)  # control is limited

    # ---- boundary conditions --------
    opti.subject_to(steering[0] == 0)   # start at position 0 ...
    opti.subject_to(velocity[0] == 0)   # start at position 0 ...

    opti.subject_to(x_pos[:,N] == x_current[:N])   # start at position
    opti.subject_to(y_pos[:,N] == y_current[:N])# start at position
    opti.subject_to(y_pos[:,N] == y_current[:N])   # start at position
    opti.subject_to(theta[0] == 0)   # start at orientation

    opti.subject_to(x_pos[N] == 6.2)  # finish line at position 100
    opti.subject_to(y_pos[N] == 4.2)  # finish line at position 15
    opti.subject_to(theta[N] == 0)  # finish line at position 1


    # ---- misc. constraints  ----------
    opti.subject_to(T >= 0)  # Time must be positive

    # ---- initial values for solver ---
    opti.set_initial(velocity, 0.1)
    opti.set_initial(T, 1)

    # ---- solve NLP              ------
    opti.solver('ipopt')  # set numerical backend
    sol = opti.solve()  # actual solve
    vel = sol.value(velocity)
    str = sol.value(steering)
    return vel, str

"""

# ---- post-processing        ------

# fig, ax = plt.subplots()
# ax.plot(sol.value(x_pos))
# ax.plot(sol.value(y_pos))
# ax.plot(sol.value(theta))
# ax.plot(sol.value(velocity))
# ax.plot(sol.value(steering))
#
# # ax.plot(limit(sol.value(pos)), 'r--')
# ax.stairs(sol.value(velocity), np.arange(0, N+1), fill=False, color='k')
# ax.stairs(sol.value(steering), np.arange(0, N + 1), fill=False, color='c')
# # ax.legend(['speed', 'pos', 'speed limit', 'throttle'], loc='northwest')
# ax.grid(True)
# #ax.box(True)
#
# plt.show()

"""

# def publish_ackermann_array():
#     # Initialize ROS node
#     rospy.init_node('Move', anonymous=True)
#
#     # Create a publisher for the AckermannDriveStamped message
#     pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
#
#     # Create a message
#     msg = AckermannDriveStamped()
#
#     # Populate the array with AckermannDriveStamped messages
#     #velocities = np.linspace(0,10,100)  # Example velocities
#     velocities, steerings = optimizer()
#     #steerings = np.linspace(0,1,10)   # Example steerings
#
#
#     for velocity, steering in zip(velocities, steerings):
#         msg.header = Header()
#         msg.header.stamp = rospy.Time.now()
#         msg.drive.speed = velocity
#         msg.drive.steering_angle = steering
#
#         # Publish the message
#         pub.publish(msg)
#
#         # Sleep for a short duration to allow time for the message to be published
#         rospy.sleep(1.0)

# if __name__ == '__main__':
#     try:
#         publish_ackermann_array()
#     except rospy.ROSInterruptException:
#         pass

def main():
    rospy.init_node('Move', anonymous=True)
    car_odometry = odom()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        position = car_odometry.pose[:3]
        x_current = position[:1]
        y_current = position[1:]

        # vel, str = optimizer(position)
        # print(vel)
        # print(str)
        # rate.sleep()

if __name__ == '__main__':
        main()
        
