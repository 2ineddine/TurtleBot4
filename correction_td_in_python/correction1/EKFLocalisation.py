import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin, cos, atan2
from scipy.linalg import sqrtm, inv

# Global variables
xTrue = None
Map = None
QTrue = None
PYTrue = None
nSteps = None
import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin, cos, atan2, sqrt
from scipy.linalg import inv

def GetOdometry(k):
    """Simulates odometry readings with noise"""
    global xTrue, QTrue
    persistent LastOdom
    
    if LastOdom is None:
        LastOdom = xTrue.copy()
    
    u = GetRobotControl(k)
    xnow = tcomp(LastOdom, u)
    
    # Add noise
    uNoise = np.random.multivariate_normal(np.zeros(3), QTrue)
    xnow = tcomp(xnow, uNoise)
    LastOdom = xnow.copy()
    return xnow

def SimulateWorld(k):
    """Updates the true robot position"""
    global xTrue
    u = GetRobotControl(k)
    xTrue = tcomp(xTrue, u)
    xTrue[2] = AngleWrap(xTrue[2])

def GetObservation(k):
    """Simulates sensor observations of map features"""
    global Map, xTrue, PYTrue, nSteps
    
    iFeature = np.random.randint(0, Map.shape[1])
    z = DoObservationModel(xTrue, iFeature, Map)
    z += np.random.multivariate_normal(np.zeros(2), PYTrue)
    z[1] = AngleWrap(z[1])
    return z, iFeature

def DoObservationModel(xVeh, iFeature, Map):
    """Observation model - range and bearing to feature"""
    Delta = Map[0:2, iFeature] - xVeh[0:2]
    z = np.array([
        np.linalg.norm(Delta),
        atan2(Delta[1], Delta[0]) - xVeh[2]
    ])
    z[1] = AngleWrap(z[1])
    return z

def DoVehicleGraphics(xEst, PEst, nSigma, Forwards):
    """Visualizes vehicle position and uncertainty"""
    ShiftTheta = atan2(Forwards[1], Forwards[0])
    h = PlotEllipse(xEst, PEst, nSigma)
    if h is not None:
        h.set_color('r')
    DrawRobot(xEst, 'b', ShiftTheta)

def DoGraphs(InnovStore, PStore, SStore, XStore, XErrStore):
    """Generates analysis plots"""
    # Innovation plot
    plt.figure(2)
    plt.subplot(2,1,1)
    plt.plot(InnovStore[0,:])
    plt.plot(SStore[0,:], 'r')
    plt.plot(-SStore[0,:], 'r')
    plt.title('Innovation')
    plt.ylabel('range')
    
    plt.subplot(2,1,2)
    plt.plot(InnovStore[1,:]*180/pi)
    plt.plot(SStore[1,:]*180/pi, 'r')
    plt.plot(-SStore[1,:]*180/pi, 'r')
    plt.ylabel('Bearing (deg)')
    plt.xlabel('time')
    
    # Error plot
    plt.figure(3)
    plt.subplot(3,1,1)
    plt.plot(XErrStore[0,:])
    plt.plot(3*PStore[0,:], 'r')
    plt.plot(-3*PStore[0,:], 'r')
    plt.title('Covariance and Error')
    plt.ylabel('x')
    
    plt.subplot(3,1,2)
    plt.plot(XErrStore[1,:])
    plt.plot(3*PStore[1,:], 'r')
    plt.plot(-3*PStore[1,:], 'r')
    plt.ylabel('y')
    
    plt.subplot(3,1,3)
    plt.plot(XErrStore[2,:]*180/pi)
    plt.plot(3*PStore[2,:]*180/pi, 'r')
    plt.plot(-3*PStore[2,:]*180/pi, 'r')
    plt.ylabel('theta (deg)')
    plt.xlabel('time')

def GetRobotControl(k):
    """Generates control inputs"""
    global nSteps
    return np.array([
        0,
        0.025,
        0.1*pi/180 * sin(3*pi*k/nSteps)
    ])

def AngleWrap(a):
    """Wrap angle to [-pi, pi] range"""
    if a > pi:
        a -= 2 * pi
    elif a < -pi:
        a += 2 * pi
    return a

def tcomp(x1, x2):
    """Compose two transformations"""
    c = cos(x1[2])
    s = sin(x1[2])
    x = x1[0] + c*x2[0] - s*x2[1]
    y = x1[1] + s*x2[0] + c*x2[1]
    theta = AngleWrap(x1[2] + x2[2])
    return np.array([x, y, theta])

def tinv(x):
    """Inverse of transformation"""
    c = cos(x[2])
    s = sin(x[2])
    x_inv = -c*x[0] - s*x[1]
    y_inv = s*x[0] - c*x[1]
    theta_inv = AngleWrap(-x[2])
    return np.array([x_inv, y_inv, theta_inv])

def EKFLocalisation():
    global xTrue, Map, QTrue, PYTrue, nSteps
    
    nSteps = 6000
    Map = 140*np.random.rand(2,30)-70
    
    # True covariance of errors
    QTrue = np.diag([0.01, 0.01, 1*pi/180])**2
    PYTrue = np.diag([2.0, 3*pi/180])**2
    
    # Modeled errors
    QEst = np.eye(3)*QTrue
    PYEst = np.eye(2)*PYTrue
    
    xTrue = np.array([1, -40, -pi/2])
    xOdomLast = GetOdometry(1)
    
    # Initial conditions
    xEst = xTrue.copy()
    PEst = np.diag([1, 1, (1*pi/180)**2])
    
    # Storage
    InnovStore = np.nan * np.zeros((2, nSteps))
    SStore = np.nan * np.zeros((2, nSteps))
    PStore = np.nan * np.zeros((3, nSteps))
    XStore = np.nan * np.zeros((3, nSteps))
    XErrStore = np.nan * np.zeros((3, nSteps))
    
    # Initial graphics
    plt.figure(1)
    plt.plot(Map[0,:], Map[1,:], 'g*')
    plt.axis('equal')
    hObsLine, = plt.plot([0,0], [0,0], 'b:')
    
    for k in range(1, nSteps):
        # World iteration
        SimulateWorld(k)
        
        # Calculate control
        xOdomNow = GetOdometry(k)
        u = tcomp(tinv(xOdomLast), xOdomNow)
        xOdomLast = xOdomNow.copy()
        
        # Prediction
        xPred = tcomp(xEst, u)
        xPred[2] = AngleWrap(xPred[2])
        PPred = A(xEst, u) @ PEst @ A(xEst, u).T + B(xEst, u) @ QEst @ B(xEst, u).T
        
        # Observe a random feature
        z, iFeature = GetObservation(k)
        
        if z is not None:
            # Predict observation
            zPred = DoObservationModel(xPred, iFeature, Map)
            
            # Get observation Jacobian
            H = GetObsJac(xPred, iFeature, Map)
            
            # Kalman update
            Innov = z - zPred
            Innov[1] = AngleWrap(Innov[1])
            
            S = H @ PPred @ H.T + PYEst
            W = PPred @ H.T @ inv(S)
            
            xEst = xPred + W @ Innov
            xEst[2] = AngleWrap(xEst[2])
            
            PEst = PPred - W @ H @ PPred
            PEst = 0.5*(PEst + PEst.T)  # Ensure symmetry
        else:
            # No observation available
            xEst = xPred.copy()
            PEst = PPred.copy()
            Innov = np.array([np.nan, np.nan])
            S = np.nan * np.eye(2)
        
        # Plot every 200 updates
        if (k-1) % 200 == 0:
            DoVehicleGraphics(xEst, PPred[0:2, 0:2], 8, np.array([0,1]))
            if z is not None:
                hObsLine.set_xdata([xEst[0], Map[0,iFeature]])
                hObsLine.set_ydata([xEst[1], Map[1,iFeature]])
            plt.pause(0.001)
        
        # Store results
        InnovStore[:, k] = Innov
        PStore[:, k] = np.sqrt(np.diag(PEst))
        SStore[:, k] = np.sqrt(np.diag(S)) if z is not None else np.array([np.nan, np.nan])
        XStore[:, k] = xEst
        XErrStore[:, k] = xTrue - xEst
    
    DoGraphs(InnovStore, PStore, SStore, XStore, XErrStore)

# Other functions (GetObservation, DoObservationModel, SimulateWorld, etc.) would be implemented similarly
# with the same structure as the MATLAB code, converting MATLAB syntax to Python/numpy equivalents

def GetObsJac(xPred, iFeature, Map):
    """h(x) Jacobian"""
    jH = np.zeros((2,3))
    Delta = Map[0:2,iFeature] - xPred[0:2]
    r = np.linalg.norm(Delta)
    jH[0,0] = -Delta[0] / r
    jH[0,1] = -Delta[1] / r
    jH[1,0] = Delta[1] / (r**2)
    jH[1,1] = -Delta[0] / (r**2)
    jH[1,2] = -1
    return jH

def A(x, u):
    """f(x,u) Jacobian wrt x"""
    s1 = sin(x[2])
    c1 = cos(x[2])
    Jac = np.array([
        [1, 0, -u[0]*s1 - u[1]*c1],
        [0, 1, u[0]*c1 - u[1]*s1],
        [0, 0, 1]
    ])
    return Jac

def B(x, u):
    """f(x,u) Jacobian wrt u"""
    s1 = sin(x[2])
    c1 = cos(x[2])
    Jac = np.array([
        [c1, -s1, 0],
        [s1, c1, 0],
        [0, 0, 1]
    ])
    return Jac

# Note: You'll need to implement the remaining functions (GetOdometry, SimulateWorld, 
# GetObservation, DoObservationModel, DoVehicleGraphics, DoGraphs) following the same pattern
# of converting MATLAB code to Python while maintaining the same function names and logic