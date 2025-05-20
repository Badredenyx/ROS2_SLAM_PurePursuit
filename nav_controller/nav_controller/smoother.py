# nav_controller/smoother.py
import numpy as np
import scipy.interpolate as si

def bspline_planning(points, resolution_factor=5):
    pts = np.array(points)
    try:
        t = range(len(pts))
        k = 2
        x_tup = si.splrep(t, pts[:,0], k=k)
        y_tup = si.splrep(t, pts[:,1], k=k)
        x_list = list(x_tup)
        y_list = list(y_tup)
        # extend knots
        x_list[1] = pts[:,0].tolist() + [0.0]*4
        y_list[1] = pts[:,1].tolist() + [0.0]*4
        ipl = np.linspace(0, len(pts)-1, len(pts)*resolution_factor)
        rx = si.splev(ipl, x_list)
        ry = si.splev(ipl, y_list)
        return list(zip(rx, ry))
    except Exception:
        return points