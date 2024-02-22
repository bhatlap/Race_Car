import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def parameterize_spline(s):
    file_path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/MexicoCity/MexicoCity_centerline.csv'
    df = pd.read_csv(file_path)
    center_line = df.iloc[:, :2].values

    x_coordinates = center_line[:, 0]
    y_coordinates = center_line[:, 1]
    t = np.cumsum(np.sqrt(np.diff(x_coordinates)**2 + np.diff(y_coordinates)**2))
    t = np.insert(t, 0, 0)  # Insert 0 at the beginning to represent the starting point
    total_length = t[-1]

    cubic_spline_x = CubicSpline(t, x_coordinates, bc_type='clamped')
    cubic_spline_y = CubicSpline(t, y_coordinates, bc_type='clamped')

    s_normalized = s % total_length

    x_point = cubic_spline_x(s_normalized)
    y_point = cubic_spline_y(s_normalized)
    psi = np.arctan(y_point,x_point)

    return x_point, y_point, psi, total_length


# Load data from CSV file


# Print the results
#print("Evaluated Point at s =", s_value, ":", (x_evaluated, y_evaluated))
x_evaluated_f = []
y_evaluated_f = []

s=2
dist_array = []
#for i in range(s):

x_1, y_1, psi, t = parameterize_spline(300)
x_2, y_2, psi, t = parameterize_spline(301)
dist = np.sqrt((x_2-x_1)**2 + (y_2-y_1)**2)
#dist_array = np.append(dist_array,dist)

print(dist)

