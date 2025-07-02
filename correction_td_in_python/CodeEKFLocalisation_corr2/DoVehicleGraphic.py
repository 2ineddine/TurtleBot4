
import numpy as np
import matplotlib.pyplot as plt
import math


def do_vehicle_graphics(x, P, n_sigma, forwards):
    """Draw vehicle graphics with covariance ellipse"""
    shift_theta = math.atan2(forwards[1], forwards[0])
    h = plot_ellipse(x, P, n_sigma)
    if h is not None:
        plt.setp(h, color='r')
    draw_robot(x, 'b', shift_theta)



def plot_ellipse(x, P, n_sigma):
    """Plot covariance ellipse"""
    eH = None
    P = P[0:2, 0:2]
    x = x[0:2]
    
    if not np.any(np.diag(P) == 0):
        D, V = np.linalg.eig(P)
        theta_vals = np.arange(0, 2*math.pi + 0.1, 0.1)
        y = n_sigma * np.array([np.cos(theta_vals), np.sin(theta_vals)])
        
        el = V @ np.sqrt(np.diag(D)) @ y
        x_tiled = np.tile(x.reshape(-1, 1), (1, el.shape[1] + 1))
        el = np.column_stack([el, el[:, 0]]) + x_tiled
        
        eH = plt.plot(el[0, :], el[1, :])
        
    return eH



def draw_robot(Xr, col, shift_theta):
    """Draw robot triangle"""
    p = 0.02
    a = plt.axis()
    l1 = (a[1] - a[0]) * p
    l2 = (a[3] - a[2]) * p
    
    P = np.array([[-1, 1, 0, -1],
                  [-1, -1, 3, -1]])
    
    theta = Xr[2] - math.pi/2 + shift_theta
    c = math.cos(theta)
    s = math.sin(theta)
    
    rotation_matrix = np.array([[c, -s], [s, c]])
    P = rotation_matrix @ P
    
    P[0, :] = P[0, :] * l1 + Xr[0]
    P[1, :] = P[1, :] * l2 + Xr[1]
    
    H = plt.plot(P[0, :], P[1, :], col, linewidth=0.1)
    plt.plot(Xr[0], Xr[1], col + '+')
    
    return H
