import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import pandas as pd

"""
def interpolate_path(path, n):
    
    # Function to use linear interpolation to create more-fine grained path data.
    # 
    # :Parameters:
    #     path - Nx2 numpy array containing the x and y coordinates of the reference path
    #     n    - number of points between 2 consecutive coordinates required
    

    x = path[:, 0]
    y = path[:, 1]

    x_interp = []
    y_interp = []

    for i in range(len(x)-1):
        x_new = np.linspace(x[i], x[i+1], n)
        y_new = np.linspace(y[i], y[i+1], n)

        x_interp = np.append(x_interp, x_new[:-1])
        y_interp = np.append(y_interp, y_new[:-1])

    x_new_e = np.linspace(x[-1], x[0], n)
    y_new_e = np.linspace(y[-1], y[0], n)

    x_interp = np.append(x_interp, x_new_e[:-1])
    y_interp = np.append(y_interp, y_new_e[:-1])

    path_interp = np.column_stack([x_interp, y_interp])
    return path_interp


"""
# if __name__ == '__main__':
#     rospack = rospkg.RosPack()
#     track = 'Silverstone'
#     pkg_path = rospack.get_path('f1tenth_simulator')
#     file_path = pkg_path + f'/scripts/Additional_maps/{track}/{track}_centerline_with_poses.csv'
#
#     df = pd.read_csv(file_path)
#     center_line = df.iloc[:, 1:3].values
#     center_line_interp = interpolate_path(center_line)
#     print(center_line[:,0].shape)
#     print(center_line_interp[:,0].shape)


def interpolate_path(path, n):

    # Function to use linear interpolation to create more-fine grained path data.
    #
    # :Parameters:
    #     path - Nx2 numpy array containing the x and y coordinates of the reference path
    #     n    - number of points between 2 consecutive coordinates required


    x = path[:, 0]
    y = path[:, 1]

    x_interp = []
    y_interp = []

    for i in range(len(x)-1):
        x_interp.extend(np.linspace(x[i], x[i+1], n, endpoint=False))
        y_interp.extend(np.linspace(y[i], y[i+1], n, endpoint=False))

    # Interpolate between the last and first points to close the loop
    x_interp.extend(np.linspace(x[-1], x[0], n, endpoint=False))
    y_interp.extend(np.linspace(y[-1], y[0], n, endpoint=False))

    path_interp = np.column_stack([x_interp, y_interp])
    return path_interp

#
# file_path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/MexicoCity/MexicoCity_centerline.csv'
#
# df = pd.read_csv(file_path)
# center_line = df.iloc[:, :2].values
#
# path = interpolate_path(center_line, 5)
# x = path[:,0]
# y = path[:,1]
# xc = center_line[:,0]
# yc= center_line[:,1]
#
# distances_c = np.sqrt(np.diff(xc)**2 + np.diff(yc)**2)
# print(distances_c)
#
# # diff_n = []
# #for i in range(len(x)-1):
# distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
# print(distances)
#
# tolerance = 1e-6
#
# # Find indices of equidistant points
# equidistant_indices = np.where(np.isclose(distances_c, distances_c[0], atol=tolerance))[0]
#
# # Filter DataFrame based on equidistant indices
# equidistant_df = df.iloc[equidistant_indices]
#
# # Print result
# if len(equidistant_df) == len(df):
#     print("All points are equidistant along the center line.")
# else:
#     print(f"Filtered DataFrame with equidistant points:\n{equidistant_df}")
#


#Generate example data
file_path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/MexicoCity/MexicoCity_centerline.csv'
df = pd.read_csv(file_path)
center_line = df.iloc[:, :2].values

x = center_line[:, 0]
y = center_line[:, 1]

# Calculate cumulative distance along the path
cumulative_distance = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
cumulative_distance = np.insert(cumulative_distance, 0, 0)

# Create a cubic spline interpolation
cubic_spline_x = CubicSpline(cumulative_distance, x)
cubic_spline_y = CubicSpline(cumulative_distance, y)

# Specify the desired number of evenly spaced points
num_points = 1
evenly_spaced_distances = np.linspace(0, cumulative_distance[-1], num_points)

# Interpolate x and y coordinates at evenly spaced distances
interpolated_x = cubic_spline_x(evenly_spaced_distances)
interpolated_y = cubic_spline_y(evenly_spaced_distances)


#Plot original and interpolated paths
plt.plot(x, y, 'o-', label='Original Path')
plt.plot(interpolated_x, interpolated_y, 's-', label='Interpolated Path')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original and Interpolated Paths')
plt.show()
