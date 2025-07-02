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
    """Range-only observation model h(x) - MODIFIED for distance only"""
    Delta = Map[:2, iFeature] - xVeh[:2]
    # Only return the range (distance), not the bearing
    z = np.linalg.norm(Delta)
    return z

def get_observation(k):
    """Get range-only observation with noise - MODIFIED"""
    global Map, xTrue, PYTrue
    
    iFeature = np.random.randint(0, Map.shape[1])
    # PYTrue is now 1x1 for range-only measurement
    z = do_observation_model(xTrue, iFeature, Map) + np.sqrt(PYTrue) * np.random.randn()
    
    return z, iFeature

def get_obs_jac(xPred, iFeature, Map):
    """Range-only observation Jacobian H - MODIFIED for 1x3 matrix"""
    jH = np.zeros((1, 3))  # 1x3 instead of 2x3
    Delta = Map[:2, iFeature] - xPred[:2]
    r = np.linalg.norm(Delta)
    
    # Only range derivatives, no bearing derivatives
    jH[0, 0] = -Delta[0] / r  # ∂r/∂x
    jH[0, 1] = -Delta[1] / r  # ∂r/∂y
    # jH[0, 2] = 0 (range doesn't depend on heading)
    
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
    """Generate result plots - MODIFIED for range-only"""
    
    # Innovation plot (only range now)
    plt.figure(2, figsize=(10, 6))
    plt.subplot(1, 1, 1)  # Only one subplot for range
    plt.plot(InnovStore[0, :], label='Range Innovation')
    plt.plot(SStore[0, :], 'r', label='+σ')
    plt.plot(-SStore[0, :], 'r', label='-σ')
    plt.title('Range-Only Innovation')
    plt.ylabel('Range (m)')
    plt.xlabel('Time Step')
    plt.legend()
    plt.grid(True)
    
    # Error and covariance plot
    plt.figure(3, figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(XErrStore[0, :], label='X Error')
    plt.plot(3*PStore[0, :], 'r', label='+3σ')
    plt.plot(-3*PStore[0, :], 'r', label='-3σ')
    plt.title('Covariance and Error (Range-Only EKF)')
    plt.ylabel('X Error (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(XErrStore[1, :], label='Y Error')
    plt.plot(3*PStore[1, :], 'r', label='+3σ')
    plt.plot(-3*PStore[1, :], 'r', label='-3σ')
    plt.ylabel('Y Error (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(XErrStore[2, :] * 180/math.pi, label='θ Error')
    plt.plot(3*PStore[2, :] * 180/math.pi, 'r', label='+3σ')
    plt.plot(-3*PStore[2, :] * 180/math.pi, 'r', label='-3σ')
    plt.ylabel('θ Error (deg)')
    plt.xlabel('Time Step')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def ekf_localisation_range_only():
    """Main EKF localization function with range-only observations"""
    global xTrue, Map, QTrue, PYTrue, nSteps, LastOdom
    
    # Initialize parameters
    nSteps = 6000
    Map = 140 * np.random.rand(2, 30) - 70
    
    # True covariance matrices
    QTrue = np.diag([0.01, 0.01, 1*math.pi/180])**2
    PYTrue = np.array([[2.0**2]])  # MODIFIED: 1x1 matrix for range-only
    
    # Estimated covariance matrices
    QEst = QTrue.copy()
    PYEst = PYTrue.copy()  # 1x1 matrix
    
    # Initial conditions
    xTrue = np.array([1, -40, -math.pi/2])
    LastOdom = None
    xOdomLast = get_odometry(1)
    
    xEst = xTrue.copy()
    PEst = np.diag([1, 1, (1*math.pi/180)**2])
    
    # Storage arrays - MODIFIED for range-only
    InnovStore = np.full((1, nSteps), np.nan)  # 1D for range only
    SStore = np.full((1, nSteps), np.nan)      # 1D for range only
    PStore = np.full((3, nSteps), np.nan)
    XStore = np.full((3, nSteps), np.nan)
    XErrStore = np.full((3, nSteps), np.nan)
    
    # Initial graphics
    plt.figure(1, figsize=(12, 8))
    plt.grid(True)
    plt.axis('equal')
    plt.plot(Map[0, :], Map[1, :], 'g*', markersize=8, label='Landmarks')
    hObsLine, = plt.plot([0, 0], [0, 0], ':', linewidth=2, label='Observation')
    plt.legend()
    plt.title('Range-Only EKF Localization')
    
    print("Starting Range-Only EKF Localization...")
    print("Observation model: Range (distance) only")
    print("Expected performance: Reduced accuracy, especially in heading estimation")
    
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
        
        # Get range-only observation
        z, iFeature = get_observation(k)
        
        if not np.isnan(z):
            # Predict observation
            zPred = do_observation_model(xPred, iFeature, Map)
            
            # Get observation Jacobian (1x3)
            H = get_obs_jac(xPred, iFeature, Map)
            
            # Kalman update
            Innov = z - zPred  # Scalar innovation
            
            S = H @ PPred @ H.T + PYEst  # 1x1 matrix
            W = PPred @ H.T / S  # 3x1 matrix (simplified inverse for scalar)
            
            xEst = xPred + W.flatten() * Innov  # Update state
            xEst[2] = angle_wrap(xEst[2])
            
            PEst = PPred - np.outer(W.flatten(), H @ PPred)
            PEst = 0.5 * (PEst + PEst.T)  # Ensure symmetry
        else:
            # No observation available
            xEst = xPred.copy()
            PEst = PPred.copy()
            Innov = np.nan
            S = np.array([[np.nan]])
        
        # Plot every 200 updates
        if (k - 2) % 200 == 0:
            do_vehicle_graphics(xEst, PEst[:2, :2], 8, [0, 1])
            if not np.isnan(z):
                hObsLine.set_xdata([xEst[0], Map[0, iFeature]])
                hObsLine.set_ydata([xEst[1], Map[1, iFeature]])
            plt.pause(0.001)
        
        # Store results
        InnovStore[0, k] = Innov
        PStore[:, k] = np.sqrt(np.diag(PEst))
        if not np.isnan(S).any():
            SStore[0, k] = np.sqrt(S[0, 0])
        XStore[:, k] = xEst
        XErrStore[:, k] = xTrue - xEst
        
        # Print progress occasionally
        if k % 1000 == 0:
            print(f"Step {k}/{nSteps}, Position error: {np.linalg.norm(xTrue[:2] - xEst[:2]):.2f}m")
    
    print("EKF completed. Generating analysis plots...")
    
    # Generate plots
    do_graphs(InnovStore, PStore, SStore, XStore, XErrStore)
    
    # Performance analysis
    final_pos_error = np.linalg.norm(xTrue[:2] - xEst[:2])
    final_heading_error = abs(angle_wrap(xTrue[2] - xEst[2])) * 180/math.pi
    
    print(f"\nFinal Performance:")
    print(f"Position Error: {final_pos_error:.2f} m")
    print(f"Heading Error: {final_heading_error:.2f} degrees")
    print(f"Final Covariance (Position): {np.sqrt(PEst[0,0]):.2f} x {np.sqrt(PEst[1,1]):.2f} m")
    print(f"Final Covariance (Heading): {np.sqrt(PEst[2,2]) * 180/math.pi:.2f} degrees")

if __name__ == "__main__":
    ekf_localisation_range_only()
