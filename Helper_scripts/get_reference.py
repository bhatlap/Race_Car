
import numpy as np
from tf.transformations import quaternion_from_euler
import rospkg
import pandas as pd


def get_reference_poses(ref_path, car_pose, ref_orientation, N,  radius=5.0):
    """
    Function to extract retrieve closest point to the car's current position that lies on the reference path.

    :Param:
        ref_path : X and Y coordinates of reference path (numpy array of dimension N x 2)
        car_position : Current X and Y coordinates of the car (numpy array of dimension 2 X 1)
    """
    car_position = car_pose[:2]
    close_pts = ref_path[np.linalg.norm(ref_path - car_position.T, axis=1) < radius]
    close_ors = ref_orientation[np.linalg.norm(ref_path - car_position.T, axis=1) < radius]

    dist = np.linalg.norm(close_pts - car_position.T, axis=1)

    idx = np.argmin(dist)

    closest_pt = np.array(close_pts[idx])
    closest_or = np.array([close_ors[idx]])

    idx_c = np.where(np.all(ref_path == closest_pt, axis=1))[0][0]

    if idx_c+N > len(ref_path)-1:

        m = len(ref_path) - idx_c
        ref_pts = np.concatenate((ref_path[idx_c:], ref_path[:(N-m)])).reshape(N, 2)
        ref_ors = np.concatenate((ref_orientation[idx_c:], ref_orientation[:(N-m)])).reshape(N, 1)
        ref_poses = np.concatenate((ref_pts, ref_ors), axis=1)

    else:
        ref_pts = ref_path[idx_c:idx_c + N].reshape(N, 2)
        ref_ors = ref_orientation[idx_c:idx_c + N].reshape(N, 1)
        ref_poses = np.concatenate((ref_pts, ref_ors), axis=1)

    return ref_poses


def create_ref_orientation(ref_path):

    x_ref = ref_path[:, 0]
    y_ref = ref_path[:, 1]
    psi_array = []

    for i in range(len(x_ref)-1):
        dx = x_ref[i + 1] - x_ref[i]
        dy = y_ref[i + 1] - y_ref[i]
        psiref = np.arctan2(dy, dx)
        psi_array = np.append(psi_array, psiref)
        q_ref = quaternion_from_euler(ai=0, aj=0, ak=psiref)

    dx_e = x_ref[0] - x_ref[-1]
    dy_e = y_ref[0] - y_ref[-1]
    psiref_e = np.arctan2(dy_e, dx_e)
    psi_array = np.append(psi_array, psiref_e)

    return np.array(psi_array)


# if __name__ == '__main__':
#
#     rospack = rospkg.RosPack()
#     track = 'Silverstone'
#     pkg_path = rospack.get_path('f1tenth_simulator')
#     file_path = pkg_path + f'/scripts/Additional_maps/{track}/{track}_centerline.csv'
#
#     df = pd.read_csv(file_path)
#     center_line = df.iloc[:, :2].values
#     center_line_interp = interpolate_path(center_line)
#     psi = create_ref_orientation(ref_path=center_line_interp)
#     print(psi.T.shape)

