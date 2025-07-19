import numpy as np
import matplotlib.pyplot as plt
from AngleWrap import AngleWrap
from tcomp import tcomp
from tinv import tinv
from DoVehicleGraphic import DoVehicleGraphics

# Global variables
xTrue = None
Map = None
QTrue = None
PYTrue = None
nSteps = None
LastOdom = None

def ensure_1d_array(arr):
    """Ensure array is 1D and properly shaped"""
    return np.asarray(arr).flatten()

def EKFLocalisation():
    global xTrue, Map, QTrue, PYTrue, nSteps
    
    plt.close('all')
    
    nSteps = 6000
    
    Map = 140*np.random.rand(2,30)-70
    
    QTrue = np.diag([0.01, 0.01, 1*np.pi/180])**2
    PYTrue = np.diag([2.0])**2
    
    QEst = np.eye(3)*QTrue
    PYEst = np.eye(1)*PYTrue
    
    xTrue = np.array([1, -40, -np.pi/2])
    xOdomLast = GetOdometry(1)
    
    xEst = xTrue.copy()
    PEst = np.diag([1, 1, 1])
    
    InnovStore = np.full((1, nSteps), np.nan)  # Changed to 1D since we only have range
    SStore = np.full((1, nSteps), np.nan)      # Changed to 1D since we only have range
    PStore = np.full((3, nSteps), np.nan)
    XStore = np.full((3, nSteps), np.nan)
    XErrStore = np.full((3, nSteps), np.nan)
    
    plt.figure(1)
    plt.grid(False)
    plt.axis('equal')
    plt.plot(Map[0,:], Map[1,:], 'g*')
    hObsLine, = plt.plot([0,0], [0,0], linestyle=':')
    
    for k in range(2, nSteps+1):
        SimulateWorld(k)
        
        xOdomNow = GetOdometry(k)
        u = tcomp(tinv(xOdomLast), xOdomNow)
        xOdomLast = xOdomNow
        
        xPred = tcomp(xEst, u)
        xPred[2] = AngleWrap(xPred[2])
        PPred = A(xEst,u) @ PEst @ A(xEst,u).T + B(xEst,u) @ QEst @ B(xEst,u).T
        
        z, iFeature = GetObservation(k)
        
        if z is not None:
            zPred = DoObservationModel(xPred, iFeature, Map)
            H = GetObsJac(xPred, iFeature, Map)
            
            Innov = z - zPred
            
            S = H @ PPred @ H.T + PYEst
            W = PPred @ H.T @ np.linalg.inv(S)
            
            xEst = xPred + W @ Innov
            xEst[2] = AngleWrap(xEst[2])
            
            PEst = PPred - W @ H @ PPred
            PEst = 0.5 * (PEst + PEst.T)
        else:
            xEst = xPred
            PEst = PPred
            Innov = np.array([np.nan])  # 1D array for range only
            S = np.full((1,1), np.nan)  # 1x1 matrix for range only
        
        if (k-2) % 200 == 0:
            DoVehicleGraphics(ensure_1d_array(xEst), PEst[:2,:2], 8, np.array([0,1]))
            if z is not None:
                hObsLine.set_xdata([xEst[0], Map[0,iFeature]])
                hObsLine.set_ydata([xEst[1], Map[1,iFeature]])
            plt.draw()
            plt.pause(0.001)
        
        # Store results - ensure proper indexing
        InnovStore[0,k-1] = Innov[0] if not np.isnan(Innov[0]) else np.nan
        PStore[:,k-1] = np.sqrt(np.diag(PEst))
        SStore[0,k-1] = np.sqrt(S[0,0]) if not np.isnan(S[0,0]) else np.nan
        XStore[:,k-1] = ensure_1d_array(xEst)
        XErrStore[:,k-1] = ensure_1d_array(xTrue) - ensure_1d_array(xEst)
    
    DoGraphs(InnovStore, PStore, SStore, XStore, XErrStore)

def DoGraphs(InnovStore, PStore, SStore, XStore, XErrStore):
    plt.figure(2)
    plt.subplot(2,1,1)
    plt.plot(InnovStore[0,:])
    plt.plot(SStore[0,:], 'r')
    plt.plot(-SStore[0,:], 'r')
    plt.title('Innovation')
    plt.ylabel('range')
    
    plt.subplot(2,1,2)
    # Since we only have range, plot zeros for bearing
    plt.plot(np.zeros_like(InnovStore[0,:]))
    plt.ylabel('Bearing (deg)')
    plt.xlabel('time')
    
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
    plt.plot(XErrStore[2,:]*180/np.pi)
    plt.plot(3*PStore[2,:]*180/np.pi, 'r')
    plt.plot(-3*PStore[2,:]*180/np.pi, 'r')
    plt.ylabel('θ')
    plt.xlabel('time')
    
    plt.show()

def GetObservation(k):
    global Map, xTrue, PYTrue
    
    iFeature = np.ceil(Map.shape[1] * np.random.rand()) - 1
    iFeature = int(iFeature)
    z = DoObservationModel(xTrue, iFeature, Map) + np.sqrt(PYTrue[0,0]) * np.random.randn()
    
    return np.array([z]), iFeature  # Return as 1D array

def DoObservationModel(xVeh, iFeature, Map):
    xVeh = ensure_1d_array(xVeh)
    Delta = Map[:2, iFeature] - xVeh[:2]
    z = np.linalg.norm(Delta)
    return z

def SimulateWorld(k):
    global xTrue
    u = GetRobotControl(k)
    xTrue = tcomp(xTrue, u)
    xTrue[2] = AngleWrap(xTrue[2])

def GetOdometry(k):
    global LastOdom, QTrue, xTrue
    
    if LastOdom is None:
        LastOdom = xTrue.copy()
    
    u = GetRobotControl(k)
    xnow = tcomp(LastOdom, u)
    uNoise = np.sqrt(np.diag(QTrue)) * np.random.randn(3)
    xnow = tcomp(xnow, uNoise)
    LastOdom = xnow
    return xnow

def GetRobotControl(k):
    global nSteps
    u = np.array([0, 0.025, 0.1*np.pi/180*np.sin(3*np.pi*k/nSteps)])
    return u

def GetObsJac(xPred, iFeature, Map):
    jH = np.zeros((1, 3))
    xPred = ensure_1d_array(xPred)
    Delta = Map[:2, iFeature] - xPred[:2]
    r = np.linalg.norm(Delta)
    jH[0, 0] = -Delta[0] / r
    jH[0, 1] = -Delta[1] / r
    return jH

def A(x, u):
    # Ensure inputs are 1D arrays
    x = ensure_1d_array(x)
    u = ensure_1d_array(u)
    s1 = np.sin(x[2])
    c1 = np.cos(x[2])
    u0 = u[0]
    u1 = u[1]
    Jac = np.array([[1.0, 0.0, -u0*s1-u1*c1],
                    [0.0, 1.0, u0*c1-u1*s1],
                    [0.0, 0.0, 1.0]])
    return Jac

def B(x, u):
    x = ensure_1d_array(x)
    s1 = np.sin(x[2])
    c1 = np.cos(x[2])
    Jac = np.array([[c1, -s1, 0.0],
                    [s1, c1, 0.0],
                    [0.0, 0.0, 1.0]])
    return Jac

if __name__ == "__main__":
    EKFLocalisation()
