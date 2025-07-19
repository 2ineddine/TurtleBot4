import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import eig
from scipy.linalg import sqrtm

def DoVehicleGraphics(x, P, n_sigma, forwards):
    """
    Main function to draw vehicle and covariance ellipse
    
    Args:
        x: state vector
        P: covariance matrix
        n_sigma: number of standard deviations for ellipse
        forwards: forward direction vector [fx, fy]
    """
    shift_theta = np.arctan2(forwards[1], forwards[0])
    h = plot_ellipse(x, P, n_sigma)
    if h is not None:
        plt.setp(h, color='r')
    draw_robot(x, 'b', shift_theta)

def plot_ellipse(x, P, n_sigma):
    """
    Plot covariance ellipse
    
    Args:
        x: state vector
        P: covariance matrix
        n_sigma: number of standard deviations
    
    Returns:
        line handle or None
    """
    # Only plot x-y part
    P = P[:2, :2]
    x = x[:2]
    
    # Check if any diagonal element is zero
    if np.any(np.diag(P) == 0):
        return None
    
    # Eigenvalue decomposition
    D, V = eig(P)
    
    # Create circle points
    theta = np.arange(0, 2*np.pi + 0.1, 0.1)
    y = n_sigma * np.array([np.cos(theta), np.sin(theta)])
    
    # Transform to ellipse
    el = V @ sqrtm(np.diag(D)) @ y
    el = el + x.reshape(-1, 1)
    
    # Close the ellipse by adding first point at the end
    el = np.column_stack([el, el[:, 0]])
    
    # Plot the ellipse
    h, = plt.plot(el[0, :], el[1, :])
    
    return h

def draw_robot(xr, col, shift_theta):
    """
    Draw robot as a triangle
    
    Args:
        xr: robot state [x, y, theta]
        col: color string
        shift_theta: additional rotation angle
    """
    p = 0.02  # percentage of axes size
    a = plt.axis()
    l1 = (a[1] - a[0]) * p
    l2 = (a[3] - a[2]) * p
    
    # Basic triangle shape
    P = np.array([[-1, 1, 0, -1],
                  [-1, -1, 3, -1]])
    
    # Rotate to point along x axis (theta = 0)
    theta = xr[2] - np.pi/2 + shift_theta
    c = np.cos(theta)
    s = np.sin(theta)
    
    # Rotation matrix
    R = np.array([[c, -s],
                  [s, c]])
    
    # Apply rotation
    P = R @ P
    
    # Scale and shift
    P[0, :] = P[0, :] * l1 + xr[0]
    P[1, :] = P[1, :] * l2 + xr[1]
    
    # Draw robot body
    plt.plot(P[0, :], P[1, :], color=col, linewidth=0.1)
    
    # Draw center point
    plt.plot(xr[0], xr[1], col + '+')

