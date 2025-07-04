
# import numpy as np
# import math
# """ 
# Q : related to the noises of wheel model
# we could estimate it using two methods -- 1. send a command from /cmd_vel and record /odom, and then
# measure the error between the expected form vs real ;

# B : Deterministic model parameters (wheel radii, baseline, gear-ratio, latency, etc)

# P : is the uncertainty about the initial position for us I think [0,0,0] is fairly enough 

# R : depending on each sensor noise  

# """

# class EKF_estimation :
#     def __init__(self):

#         # Initial robot state: [x, y, theta]
#         self.x_est = np.array([0.0, 0.0, 0.0])

#         # Initial covariance
#         self.P_est = np.diag([0.5, 0.5, np.deg2rad(10)])**2

#         # Process noise (motion uncertainty)
#         self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])**2

#         # Measurement noise (depends on sensor)
#         self.R = np.diag([0.5, np.deg2rad(5)])**2

#         #self.Bx = np.diag([0.1, 0.1, np.deg2rad(1)])**2 #simulated


#     def predict_ekf(self, x, P, u, dt):
#         v, omega = u # get the vector command 
#         theta = x[2]  # get theta from the last prediction 

#         # Predict state
#         # after computing the new x+ we must update the vector state 

#         x_pred = x.copy()
#         x_pred[0] += v * math.cos(theta) * dt
#         x_pred[1] += v * math.sin(theta) * dt
#         x_pred[2] += omega * dt
#         x_pred[2] = (x_pred[2] + np.pi) % (2 * np.pi) - np.pi  # wrap

#         # update Jacobian 
#         Fx = np.eye(3)
#         Fx[0, 2] = -v * math.sin(theta) * dt
#         Fx[1, 2] =  v * math.cos(theta) * dt

#         Bx = self.B(theta, dt)
#         # Predict covariance
#         P_pred = Fx @ P @ Fx.T + (Bx @ self.Q @ Bx.T)

#         return x_pred, P_pred

#     def update_ekf(self, x_pred, P_pred, z_meas, landmark_pos, R):
#         # Predict observation
#         z_pred,H = self.ekf_observation_model(x_pred, landmark_pos) # from the next function we get the z_pred and its covariance
#         #z_pred is computed only by the predicted value (so we have the landmark position and then comp)
#         y = z_meas - z_pred # compute the innovation
#         y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # wrap bearing

#         # Jacobian

#         # Kalman gain
#         S = H @ P_pred @ H.T + R # compute the innovation's covariance 
#         K = P_pred @ H.T @ np.linalg.inv(S) # compute the Kalman gain 

#         # Update
#         x_upd = x_pred + K @ y # update the best prediction 
#         x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi  # wrap 

#         P_upd = (np.eye(3) - K @ H) @ P_pred #update the covariance prediction 

#         return x_upd, P_upd, y 


#     def ekf_observation_model(self,x, landmark_pos):
#         """
#         Compute the expected observation (range and bearing)
#         and its Jacobian with respect to the robot state.

#         Parameters:
#             x: robot state [x, y, theta]
#             landmark_pos: landmark position [x_l, y_l]

#         Returns:
#             z_pred: predicted observation [range, bearing]
#             H: Jacobian matrix (2x3)
#         """

#         dx = landmark_pos[0] - x[0] # the distance between the robot x' coordinate and landmark x center's coordinate 
#         dy = landmark_pos[1] - x[1] # the distance between the robot y' coordinate and landmark y center's coordinate

#         r2 = dx**2 + dy**2 
#         r = math.sqrt(r2)  # compute the distance between the robot and landmark d = sqrt (x**2 + y**2)
#         phi = math.atan2(dy, dx) - x[2] # compute the azimute angle between the angle and landmark
#         phi = (phi + np.pi) % (2 * np.pi) - np.pi  # angle wrap [-π, π]

#         # z_pred is the equivalent of  H*x- in my notebook so to be verfied 
#         z_pred = np.array([r, phi]) # used to  compute the innovation v= z_meas -(H*x-)
        

#         # Jacobian H (from theorical equation)
#         H = np.array([
#             [-dx / r,    -dy / r,     0],
#             [ dy / r2,   -dx / r2,   -1]
#         ])

#         return z_pred, H

#     def B_matrix (self, x_t):
#         """ B = ∂f/∂u at this pose and time step """
#         c, s = math.cos(x_t[2]), math.sin(x_t[2])
#         return np.array([[c, -s, 0],
#                         [s,  c, 0],
#                         [0,       0,  1]])

#     def A_matrix (self,x_t,x_u):
#         A = np.eye(3)
#         A[0, 2] = -x_u[0]*math.sin(x_t[2]) - x_u[1]*math.cos(x_t[2])
#         A[1, 2] =  x_u[0] * math.cos(x_t[2])-x_u[1]*math.sin(x_t[2])
#         return A 

import numpy as np
import math

"""
Updated EKF implementation with:
1. Odometry-based motion model using U = [xu, yu, thetau] (odometry data)
2. Camera-based landmark observations z_k = [rk, phik] (range and bearing)

State vector: x_t = [xt, yt, thetat]
Control input: U = [xu, yu, thetau] (odometry displacements)
Observations: z_k = [rk, phik] (range and bearing from camera)

Q : Motion model noise covariance (for odometry uncertainties)
P : Initial state uncertainty covariance  
R : Measurement noise covariance (camera measurements)
"""

class EKF_estimation:
    def __init__(self):
        # Initial robot state: x_t = [xt, yt, thetat]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # Initial covariance
        self.P_est = np.diag([0.5, 0.5, np.deg2rad(10)])**2

        # Process noise (odometry uncertainty)
        # Represents uncertainty in U = [xu, yu, thetau] measurements
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])**2

        # Measurement noise (camera measurements z_k = [rk, phik])
        #self.R = np.diag([0.5, np.deg2rad(5)])**2
        self.R = np.array([[np.deg2rad(5)**2]]) 

    def predict_ekf(self, x_t, P, U):
        """
        Prediction step using odometry-based motion model
        
        Parameters:
            x_t: previous state [xt, yt, thetat]
            P: previous covariance matrix
            U: odometry command [xu, yu, thetau]
        
        Returns:
            x_pred: predicted state
            P_pred: predicted covariance
        """
        xu, yu, thetau = U 
        thetat = x_t[2]
        
        # Motion model: transform odometry measurements to global frame
        cos_theta = math.cos(thetat)
        sin_theta = math.sin(thetat)
        
        # Predict state using odometry measurements
        x_pred = x_t.copy()
        x_pred[0] += xu * cos_theta - yu * sin_theta
        x_pred[1] += xu * sin_theta + yu * cos_theta
        x_pred[2] += thetau
        x_pred[2] = (x_pred[2] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Jacobian of motion model with respect to state (Fx)
        Fx = np.eye(3)
        Fx[0, 2] = -xu * sin_theta - yu * cos_theta
        Fx[1, 2] = xu * cos_theta - yu * sin_theta
        
        # Jacobian of motion model with respect to control input (Fu)
        Fu = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        
        # Predict covariance
        P_pred = Fx @ P @ Fx.T + Fu @ self.Q @ Fu.T
        
        return x_pred, P_pred

    def update_ekf(self, x_pred, P_pred, z_k, landmark_pos, R):
        """
        Update step using camera measurements z_k = [rk, phik]
        
        Parameters:
            x_pred: predicted state
            P_pred: predicted covariance
            z_k: camera measurement [rk, phik] (range and bearing)
            landmark_pos: landmark position [x_l, y_l]
            R: measurement noise covariance (2x2 matrix)
        
        Returns:
            x_upd: updated state
            P_upd: updated covariance
            y: innovation
        """
        # Predict observation z_k = [rk, phik]
        z_pred, H = self.ekf_observation_model(x_pred, landmark_pos)
        
        # Innovation
        y = z_k - z_pred
        #y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # wrap bearing angle
        y = (y + np.pi) % (2 * np.pi) - np.pi
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Update state
        x_upd = x_pred + K @ y
        x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Update covariance
        P_upd = (np.eye(3) - K @ H) @ P_pred
        
        return x_upd, P_upd, y

    def ekf_observation_model(self, x_t, landmark_pos):
        """
        Compute the expected camera observation z_k = [rk, phik] and its Jacobian
        
        Parameters:
            x_t: robot state [xt, yt, thetat]
            landmark_pos: landmark position [x_l, y_l]
        
        Returns:
            z_pred: predicted observation [rk, phik]
            H: Jacobian matrix (2x3)
        """
        xt, yt, thetat = x_t
        
        dx = landmark_pos[0] - xt
        dy = landmark_pos[1] - yt
        
        r2 = dx**2 + dy**2
        rk = math.sqrt(r2)  # range
        phik = math.atan2(dy, dx) - thetat  # bearing relative to robot orientation
        phik = (phik + np.pi) % (2 * np.pi) - np.pi  # wrap angle [-π, π]
        
        # Predicted observation z_k = [rk, phik]
        #z_pred = np.array([rk, phik])

        # Predicted observation z_k = [phik] (bearing only)
        z_pred = phik  # scalar
        
        # Jacobian H (2x3 matrix)
        # if rk > 1e-6:  # Avoid division by zero
        #     H = np.array([
        #         [-dx / rk,    -dy / rk,     0],
        #         [ dy / r2,   -dx / r2,   -1]
        #     ])
        # else:
        #     print ("error the distance between estimated ~ 0m, the code uses the alternative matrix...")
        #     H = np.array([
        #         [0, 0, 0],
        #         [0, 0, -1]
        #     ])
        if r2 > 1e-12:  # Avoid division by zero
            H = np.array([[dy / r2, -dx / r2, -1]])
        else:
            H = np.array([[0, 0, -1]])
        
        return z_pred, H

    def run_ekf_step(self, U, z_k, landmark_pos):
        """
        Complete EKF step (predict + update)
        
        Parameters:
            U: odometry command [xu, yu, thetau]
            z_k: camera measurement [rk, phik] or None if no measurement
            landmark_pos: landmark position [x_l, y_l]
        
        Returns:
            x_est: updated state estimate
            P_est: updated covariance estimate
        """
        # Prediction step
        x_pred, P_pred = self.predict_ekf(self.x_est, self.P_est, U)
        
        # Update step (if measurement is available)
        if z_k is not None:
            self.x_est, self.P_est, innovation = self.update_ekf(
                x_pred, P_pred, z_k, landmark_pos, self.R
            )
        else:
            # No measurement available, use prediction only
            self.x_est = x_pred
            self.P_est = P_pred
        
        return self.x_est, self.P_est

    # def B_matrix(self, x_t):
    #     """
    #     B = ∂f/∂U at current pose (Jacobian w.r.t. control input)
    #     """
    #     thetat = x_t[2]
    #     cos_theta = math.cos(thetat)
    #     sin_theta = math.sin(thetat)
        
    #     return np.array([
    #         [cos_theta, -sin_theta, 0],
    #         [sin_theta, cos_theta, 0],
    #         [0, 0, 1]
    #     ])

    # def A_matrix(self, x_t, U):
    #     """
    #     A = ∂f/∂x at current pose and control input (Jacobian w.r.t. state)
    #     """
    #     xu, yu, thetau = U
    #     thetat = x_t[2]
        
    #     A = np.eye(3)
    #     A[0, 2] = -xu * math.sin(thetat) - yu * math.cos(thetat)
    #     A[1, 2] = xu * math.cos(thetat) - yu * math.sin(thetat)
        
    #     return A

    def get_state_estimate(self):
        """Return current state estimate x_t = [xt, yt, thetat]"""
        return self.x_est.copy()
    
    def get_covariance_estimate(self):
        """Return current covariance estimate"""
        return self.P_est.copy()

# Example usage and testing
if __name__ == "__main__":
    # Initialize EKF
    ekf = EKF_estimation()
    
    # Example odometry command U = [xu, yu, thetau]
    # Robot moved 1m forward, 0.5m left, rotated 0.1 rad
    U_odometry = np.array([1.0, 0.5, 0.1])
    
    # Example camera measurement z_k = [rk, phik] to landmark at (5, 3)
    landmark_position = np.array([5.0, 3.0])
    #z_k_camera = np.array([4.8, np.deg2rad(30)])  # 4.8m range, 30° bearing
    z_k_camera = np.deg2rad(30)
    # Run one EKF step
    x_est, P_est = ekf.run_ekf_step(U_odometry, z_k_camera, landmark_position)
    
    print(f"State estimate x_t: xt={x_est[0]:.3f}, yt={x_est[1]:.3f}, thetat={np.rad2deg(x_est[2]):.1f}°")
    print(f"Position uncertainty: σx={np.sqrt(P_est[0,0]):.3f}, σy={np.sqrt(P_est[1,1]):.3f}")
    print(f"Orientation uncertainty: σθ={np.rad2deg(np.sqrt(P_est[2,2])):.1f}°")