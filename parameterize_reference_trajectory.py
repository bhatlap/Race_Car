import numpy as np
from scipy.interpolate import interp1d


def arc_length_parameterize(path):

    x_data = path[:, 0]
    y_data = path[:, 1]

    arc_lengths = np.cumsum(np.sqrt(np.diff(x_data)**2 + np.diff(y_data)**2))
    xd_e = x_data[-1] - x_data[0]
    yd_e = y_data[-1] - y_data[0]
    arclength_e = arc_lengths[-1] + np.sqrt(xd_e**2 + yd_e**2)

    arc_lengths = np.append(arc_lengths, arclength_e)

    x_ref = interp1d(arc_lengths, x_data, kind='quadratic', fill_value='extrapolate')
    y_ref = interp1d(arc_lengths, y_data, kind='quadratic', fill_value='extrapolate')

    return x_ref, y_ref


