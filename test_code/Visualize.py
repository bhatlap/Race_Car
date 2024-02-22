import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from trackParser import get_bounds
def generate_bounds(centre_line, dist, t_samp = 1.0):

    left_bound_traj = Path()
    left_bound_traj.header = Header()
    left_bound_traj.header.frame_id = 'map'
    left_bound_traj.header.stamp = rospy.Time(0)

    right_bound_traj = Path()
    right_bound_traj.header = Header()
    right_bound_traj.header.frame_id = 'map'
    right_bound_traj.header.stamp = rospy.Time(0)

    left_bounds , right_bounds = get_bounds(centre_line, dist)

    x_left = left_bounds[:,0]
    y_left = left_bounds[:,1]

    x_right = right_bounds[:,0]
    y_right = right_bounds[:,1]

    timestamps = np.arange(0,len(x_left),1)
    time_interp = np.arange(timestamps[0], timestamps[-1], t_samp)

    x_left_bound = np.interp(time_interp,timestamps,x_left)
    y_left_bound = np.interp(time_interp,timestamps,y_left)

    x_right_bound = np.interp(time_interp,timestamps,x_right)
    y_right_bound = np.interp(time_interp,timestamps,y_right)

    #plt.plot(x_right_bound,y_right_bound)
    #plt.plot(x_left_bound,y_left_bound)
    #plt.plot(x_right_bound,y_right_bound)
    #plt.plot(x_left_bound,y_left_bound)
    #plt.show()

def main():


    file_path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/Silverstone/Silverstone_centerline.csv'

    df = pd.read_csv(file_path)
    center_line = df.iloc[0::2, :2].values
    center_line = np.vstack([center_line,center_line[:2,:]])

    d = df.iloc[0,3]
    #print(center_line)
    generate_bounds(center_line,d,t_samp = 0.02)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")