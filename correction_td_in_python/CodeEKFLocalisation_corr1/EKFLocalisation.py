import numpy as np
import matplotlib.pyplot as plt
import math
import time
from DoVehicleGraphic import *
from tcomp import *
from tinv import * 
from AngleWrap import *
# Global variables
xTrue = None
Map = None
QTrue = None
PYTrue = None
nSteps = None
LastOdom = None

def get_robot_control(k):
    """Generate robot control commands"""
    global nSteps
    u = np.array([0, 0.025, 0.1*math.pi/180*math.sin(3*math.pi*k/nSteps)])
    return u

def get_odometry(k):
    """Get odometry measurements with noise"""
    global LastOdom, QTrue, xTrue
    
    if LastOdom is None:
        LastOdom = xTrue.copy()
    
    u = get_robot_control(k)
    xnow = tcomp(LastOdom, u)
    uNoise = np.sqrt(QTrue) @ np.random.randn(3)
    xnow = tcomp(xnow, uNoise)
    LastOdom = xnow.copy()
    
    return xnow

def simulate_world(k):
    """Simulate world dynamics"""
    global xTrue
    u = get_robot_control(k)
    xTrue = tcomp(xTrue, u)
    xTrue[2] = angle_wrap(xTrue[2])

def do_observation_model(xVeh, iFeature, Map):
    """Observation model h(x)"""
    Delta = Map[:2, iFeature] - xVeh[:2]
    z = np.array([
        np.linalg.norm(Delta),
        math.atan2(Delta[1], Delta[0]) - xVeh[2]
    ])
    z[1] = angle_wrap(z[1])
    return z

def get_observation(k):
    """Get observation with noise"""
    global Map, xTrue, PYTrue
    
    iFeature = np.random.randint(0, Map.shape[1])
    z = do_observation_model(xTrue, iFeature, Map) + np.sqrt(PYTrue) @ np.random.randn(2)
    z[1] = angle_wrap(z[1])
    
    return z, iFeature

def get_obs_jac(xPred, iFeature, Map):
    """Observation Jacobian H"""
    jH = np.zeros((2, 3))
    Delta = Map[:2, iFeature] - xPred[:2]
    r = np.linalg.norm(Delta)
    
    jH[0, 0] = -Delta[0] / r
    jH[0, 1] = -Delta[1] / r
    jH[1, 0] = Delta[1] / (r**2)
    jH[1, 1] = -Delta[0] / (r**2)
    jH[1, 2] = -1
    
    return jH

def A(x, u):
    """State transition Jacobian with respect to state"""
    s1 = math.sin(x[2])
    c1 = math.cos(x[2])
    
    Jac = np.array([
        [1, 0, -u[0]*s1 - u[1]*c1],
        [0, 1,  u[0]*c1 - u[1]*s1],
        [0, 0,  1]
    ])
    
    return Jac

def B(x, u):
    """State transition Jacobian with respect to control"""
    s1 = math.sin(x[2])
    c1 = math.cos(x[2])
    
    Jac = np.array([
        [c1, -s1, 0],
        [s1,  c1, 0],
        [0,   0,  1]
    ])
    
    return Jac

def do_graphs(InnovStore, PStore, SStore, XStore, XErrStore):
    """Generate result plots"""
    
    # Innovation plot
    plt.figure(2, figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(InnovStore[0, :])
    plt.plot(SStore[0, :], 'r')
    plt.plot(-SStore[0, :], 'r')
    plt.title('Innovation')
    plt.ylabel('range')
    
    plt.subplot(2, 1, 2)
    plt.plot(InnovStore[1, :] * 180/math.pi)
    plt.plot(SStore[1, :] * 180/math.pi, 'r')
    plt.plot(-SStore[1, :] * 180/math.pi, 'r')
    plt.ylabel('Bearing (deg)')
    plt.xlabel('time')
    
    # Error and covariance plot
    plt.figure(3, figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(XErrStore[0, :])
    plt.plot(3*PStore[0, :], 'r')
    plt.plot(-3*PStore[0, :], 'r')
    plt.title('Covariance and Error')
    plt.ylabel('x')
    
    plt.subplot(3, 1, 2)
    plt.plot(XErrStore[1, :])
    plt.plot(3*PStore[1, :], 'r')
    plt.plot(-3*PStore[1, :], 'r')
    plt.ylabel('y')
    
    plt.subplot(3, 1, 3)
    plt.plot(XErrStore[2, :] * 180/math.pi)
    plt.plot(3*PStore[2, :] * 180/math.pi, 'r')
    plt.plot(-3*PStore[2, :] * 180/math.pi, 'r')
    plt.ylabel('θ (deg)')
    plt.xlabel('time')
    
    plt.tight_layout()
    plt.show()

def ekf_localisation():
    """Main EKF localization function"""
    global xTrue, Map, QTrue, PYTrue, nSteps, LastOdom
    
    # Initialize parameters
    nSteps = 6000
    Map = 140 * np.random.rand(2, 30) - 70
    
    # True covariance matrices
    QTrue = np.diag([0.01, 0.01, 1*math.pi/180])**2
    PYTrue = np.diag([2.0, 3*math.pi/180])**2
    
    # Estimated covariance matrices
    QEst = QTrue.copy()
    PYEst = PYTrue.copy()
    
    # Initial conditions
    xTrue = np.array([1, -40, -math.pi/2])
    LastOdom = None
    xOdomLast = get_odometry(1)
    
    xEst = xTrue.copy()
    PEst = np.diag([1, 1, (1*math.pi/180)**2])
    
    # Storage arrays
    InnovStore = np.full((2, nSteps), np.nan)
    SStore = np.full((2, nSteps), np.nan)
    PStore = np.full((3, nSteps), np.nan)
    XStore = np.full((3, nSteps), np.nan)
    XErrStore = np.full((3, nSteps), np.nan)
    
    # Initial graphics
    plt.figure(1, figsize=(12, 8))
    plt.hold = True
    plt.grid(False)
    plt.axis('equal')
    plt.plot(Map[0, :], Map[1, :], 'g*')
    hObsLine, = plt.plot([0, 0], [0, 0], ':')
    
    # Main EKF loop
    for k in range(2, nSteps):
        # Simulate world
        simulate_world(k)
        
        # Calculate control input
        xOdomNow = get_odometry(k)
        u = tcomp(tinv(xOdomLast), xOdomNow)
        xOdomLast = xOdomNow.copy()
        
        # Prediction step
        xPred = tcomp(xEst, u)
        xPred[2] = angle_wrap(xPred[2])
        
        A_jac = A(xEst, u)
        B_jac = B(xEst, u)
        PPred = A_jac @ PEst @ A_jac.T + B_jac @ QEst @ B_jac.T
        
        # Get observation
        z, iFeature = get_observation(k)
        
        if not np.isnan(z).any():
            # Predict observation
            zPred = do_observation_model(xPred, iFeature, Map)
            
            # Get observation Jacobian
            H = get_obs_jac(xPred, iFeature, Map)
            
            # Kalman update
            Innov = z - zPred
            Innov[1] = angle_wrap(Innov[1])
            
            S = H @ PPred @ H.T + PYEst
            W = PPred @ H.T @ np.linalg.inv(S)
            
            xEst = xPred + W @ Innov
            xEst[2] = angle_wrap(xEst[2])
            
            PEst = PPred - W @ H @ PPred
            PEst = 0.5 * (PEst + PEst.T)  # Ensure symmetry
        else:
            # No observation available
            xEst = xPred.copy()
            PEst = PPred.copy()
            Innov = np.array([np.nan, np.nan])
            S = np.full((2, 2), np.nan)
        
        # Plot every 200 updates
        if (k - 2) % 200 == 0:
            do_vehicle_graphics(xEst, PEst[:2, :2], 8, [0, 1])
            if not np.isnan(z).any():
                hObsLine.set_xdata([xEst[0], Map[0, iFeature]])
                hObsLine.set_ydata([xEst[1], Map[1, iFeature]])
            plt.pause(0.001)
        
        # Store results
        InnovStore[:, k] = Innov
        PStore[:, k] = np.sqrt(np.diag(PEst))
        if not np.isnan(S).any():
            SStore[:, k] = np.sqrt(np.diag(S))
        XStore[:, k] = xEst
        XErrStore[:, k] = xTrue - xEst
    
    # Generate plots
    do_graphs(InnovStore, PStore, SStore, XStore, XErrStore)

if __name__ == "__main__":
    ekf_localisation()