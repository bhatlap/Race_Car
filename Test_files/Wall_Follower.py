#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan as lscan
from ackermann_msgs.msg import AckermannDriveStamped as acker

#PID CONTROL PARAMS
kp = 10
kd = 0
ki = 0.1
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs


        self.lidar_sub = rospy.Subscriber('/scan', lscan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/drive', acker, queue_size=10)



    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.

        #Convert to radians
        angle = math.radians(angle)
        angle = (angle + 2 * math.pi) % (2 * math.pi)

        r = 0
        min_angle = data.angle_min
        max_angle = data.angle_max
        inc_angle = data.angle_increment

        total_steps = round((max_angle - min_angle)/inc_angle)
        index_at_angle = round(((angle - min_angle)/inc_angle)%total_steps)
        r = data.ranges[int(index_at_angle)]

        return r

    def pid_control(self, error, dt):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for

        P = kp*error

        integral += error * dt
        I = ki*integral

        derivative = (error - prev_error)/dt
        D = kd*derivative

        prev_error = error

        angle = P + I + D

        return angle


    def followLeft(self, Dt, leftDist):
        #Follow left wall as per the algorithm 
        difference = leftDist - Dt
        return difference

    def get_alpha(self, a,b, theta):

        theta = math.radians(theta)
        alpha = math.atan((a*math.cos(theta) - b)/a*math.sin(theta))

        return alpha

    def get_Dt(self,alpha,b,L):

        Dt = b * math.cos(alpha) + L * math.sin(alpha)

        return Dt

    def lidar_callback(self, data):
        """ 
        """
        b = self.getRange(data, -90)
        a = self.getRange(data, -60)

        alpha = self.get_alpha(a,b,30)

        Dt = self.get_Dt(alpha,b,5)
        error = self.followLeft(Dt,1)#TODO: replace with error returned by followLeft


        #send error to pid_control
        steering = self.pid_control(error, 0.01)
        if math.radians(0) < steering < math.radians(10):
            velocity = 2.0
        elif math.radians(10) < steering < math.radians(20):
            velocity = 1.0
        else: velocity = 0.5

        drive_msg = acker()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
