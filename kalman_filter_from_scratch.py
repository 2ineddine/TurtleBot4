
import numpy as np
import math
""" 
Q : related to the noises of wheel model
we could estimate it using two methods -- 1. send a command from /cmd_vel and record /odom, and then
measure the error between the expected form vs real ;

B : Deterministic model parameters (wheel radii, baseline, gear-ratio, latency, etc)

P : is the uncertainty about the initial position for us I think [0,0,0] is fairly enough 

R : depending on each sensor noise  

"""

class EKF_estimation :
    def __init__(self):

        # Initial robot state: [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # Initial covariance
        self.P_est = np.diag([0.5, 0.5, np.deg2rad(10)])**2

        # Process noise (motion uncertainty)
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])**2

        # Measurement noise (depends on sensor)
        self.R = np.diag([0.5, np.deg2rad(5)])**2

        #self.Bx = np.diag([0.1, 0.1, np.deg2rad(1)])**2 #simulated


    def predict_ekf(self, x, P, u, dt):
        v, omega = u # get the vector command 
        theta = x[2]  # get theta from the last prediction 

        # Predict state
        # after computing the new x+ we must update the vector state 

        x_pred = x.copy()
        x_pred[0] += v * math.cos(theta) * dt
        x_pred[1] += v * math.sin(theta) * dt
        x_pred[2] += omega * dt
        x_pred[2] = (x_pred[2] + np.pi) % (2 * np.pi) - np.pi  # wrap

        # update Jacobian 
        Fx = np.eye(3)
        Fx[0, 2] = -v * math.sin(theta) * dt
        Fx[1, 2] =  v * math.cos(theta) * dt

        Bx = self._ctrl_jacobian(theta, dt)
        # Predict covariance
        P_pred = Fx @ P @ Fx.T + (Bx @ self.Q @ Bx.T)

        return x_pred, P_pred

    def update_ekf(self, x_pred, P_pred, z_meas, landmark_pos, R):
        # Predict observation
        z_pred,H = self.ekf_observation_model(x_pred, landmark_pos) # from the next function we get the z_pred and its covariance
        #z_pred is computed only by the predicted value (so we have the landmark position and then comp)
        y = z_meas - z_pred # compute the innovation
        y[1] = (y[1] + np.pi) % (2 * np.pi) - np.pi  # wrap bearing

        # Jacobian

        # Kalman gain
        S = H @ P_pred @ H.T + R # compute the innovation's covariance 
        K = P_pred @ H.T @ np.linalg.inv(S) # compute the Kalman gain 

        # Update
        x_upd = x_pred + K @ y # update the best prediction 
        x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi  # wrap 

        P_upd = (np.eye(3) - K @ H) @ P_pred #update the covariance prediction 

        return x_upd, P_upd, y 


    def ekf_observation_model(self,x, landmark_pos):
        """
        Compute the expected observation (range and bearing)
        and its Jacobian with respect to the robot state.

        Parameters:
            x: robot state [x, y, theta]
            landmark_pos: landmark position [x_l, y_l]

        Returns:
            z_pred: predicted observation [range, bearing]
            H: Jacobian matrix (2x3)
        """

        dx = landmark_pos[0] - x[0] # the distance between the robot x' coordinate and landmark x center's coordinate 
        dy = landmark_pos[1] - x[1] # the distance between the robot y' coordinate and landmark y center's coordinate

        r2 = dx**2 + dy**2 
        r = math.sqrt(r2)  # compute the distance between the robot and landmark d = sqrt (x**2 + y**2)
        phi = math.atan2(dy, dx) - x[2] # compute the azimute angle between the angle and landmark
        phi = (phi + np.pi) % (2 * np.pi) - np.pi  # angle wrap [-π, π]

        # z_pred is the equivalent of  H*x- in my notebook so to be verfied 
        z_pred = np.array([r, phi]) # used to  compute the innovation v= z_meas -(H*x-)
        

        # Jacobian H (from theorical equation)
        H = np.array([
            [-dx / r,    -dy / r,     0],
            [ dy / r2,   -dx / r2,   -1]
        ])

        return z_pred, H

    def _ctrl_jacobian(self, theta, dt):
        """ B = ∂f/∂u at this pose and time step """
        c, s = math.cos(theta), math.sin(theta)
        return np.array([[c*dt, -s*dt, 0],
                        [s*dt,  c*dt, 0],
                        [0,       0,  dt]])
