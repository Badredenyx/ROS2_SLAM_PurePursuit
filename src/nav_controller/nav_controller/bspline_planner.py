import numpy as np
from scipy.interpolate import splprep, splev

def smooth_path(path: list, num_points=200, degree=3) -> list:
    """
    Fit a B‑spline to the input path and sample it uniformly.
    path: list of (row,col) grid coords
    Returns: list of (row_f, col_f) floats in continuous space
    """
    # Extract x (col) and y (row) separately
    x = [p[1] for p in path]
    y = [p[0] for p in path]

    # splprep: parametric B‑spline representation
    # s=0: exact fit through points; k=degree
    tck, u = splprep([x, y], s=0, k=degree)

    # uniform parameter samples from 0 to 1
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)

    # Return list of (row, col)
    return list(zip(y_new, x_new))