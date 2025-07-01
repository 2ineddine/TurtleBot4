import numpy as np
import matplotlib.pyplot as plt
from math import atan2, cos, sin, pi
from scipy.linalg import sqrtm

def AngleWrap(a):
    """Wrap angle to [-pi, pi] range"""
    if a > pi:
        a -= 2 * pi
    elif a < -pi:
        a += 2 * pi
    return a

def DoVehicleGraphics(x, P, nSigma, Forwards):
    ShiftTheta = atan2(Forwards[1], Forwards[0])  # Note: y,x order in Python
    h = PlotEllipse(x, P, nSigma)
    if h is not None:
        h.set_color('r')
    DrawRobot(x, 'b', ShiftTheta)

def PlotEllipse(x, P, nSigma):
    eH = None
    P = P[0:2, 0:2]  # only plot x-y part
    x = x[0:2]
    
    if not np.any(np.diag(P) == 0):
        D, V = np.linalg.eig(P)
        angles = np.arange(0, 2*pi+0.1, 0.1)
        y = nSigma * np.array([np.cos(angles), np.sin(angles)])
        el = V @ np.diag(np.sqrt(D)) @ y
        el = np.column_stack((el, el[:, 0])) + np.tile(x, (el.shape[1], 1)).T
        eH, = plt.plot(el[0, :], el[1, :])
    return eH

def DrawRobot(Xr, col, ShiftTheta):
    p = 0.02  # percentage of axes size
    a = plt.axis()
    l1 = (a[1] - a[0]) * p
    l2 = (a[3] - a[2]) * p
    
    P = np.array([[-1, 1, 0, -1],
                 [-1, -1, 3, -1]])
    
    theta = AngleWrap(Xr[2] - pi/2 + ShiftTheta)
    c, s = cos(theta), sin(theta)
    P = np.array([[c, -s], [s, c]]) @ P
    
    P[0, :] = P[0, :] * l1 + Xr[0]
    P[1, :] = P[1, :] * l2 + Xr[1]
    
    plt.plot(P[0, :], P[1, :], color=col, linewidth=0.1)
    plt.plot(Xr[0], Xr[1], f'{col}+')