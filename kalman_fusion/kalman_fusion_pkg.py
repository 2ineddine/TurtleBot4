import numpy as np

class kalman_fusion:
    def __init__(self, p0, q, r, x0=[0, 0, 0]):
        self.p0 = np.array(p0)  # Initial covariance (3x3)
        self.p1_ = None         # Predicted covariance
        self.p1 = None          # Updated covariance
        self.q = np.array(q)    # Process noise (3x3)
        self.r = np.array(r)    # Measurement noise (2x2 or 3x3)
        self.x0 = np.array(x0)  # Initial state [x, y, θ]
        self.x1_ = None         # Predicted state
        self.x1 = None          # Updated state
        self.u = None           # Control input [v, ω]
        self.y = None           # Measurement residual
        self.jacobian = None    # Jacobian matrix (3x3)
        self.delta_t = None     # Time step

    def first_prediction(self, u1, dt):
        self.delta_t = dt
        self.u = np.array(u1)  # [v, ω]
        self.x1_ = self.x0.copy()
        self.x1_[0] = self.x0[0] + (self.u[0] * self.delta_t * np.cos(self.x0[2]))
        self.x1_[1] = self.x0[1] + (self.u[0] * self.delta_t * np.sin(self.x0[2]))
        self.x1_[2] = self.x0[2] + (self.u[1] * self.delta_t)
        return self.x1_

    def compute_jacobian(self):
        self.jacobian = np.eye(3)  # Fixed: Now 3x3 identity by default
        self.jacobian[0, 2] = -self.u[0] * self.delta_t * np.sin(self.x0[2])  # Fixed: self.u (not u1)
        self.jacobian[1, 2] = self.u[0] * self.delta_t * np.cos(self.x0[2])
        return self.jacobian

    def compute_covariance(self):
        self.p1_ = self.jacobian @ self.p0 @ self.jacobian.T + self.q  # Fixed: @ operator for matrix multiply
        return self.p1_

    def measure_residual(self, z):
        self.y = np.array(z) - self.x1_[:len(z)]  # Fixed: Handle z of size 2 or 3
        return self.y

    def innovation_covariance(self):
        return self.p1_[:len(self.y), :len(self.y)] + self.r  # Fixed: Match dimensions of y

    def kalman_gain(self):
        return self.p1_[:, :len(self.y)] @ np.linalg.inv(self.innovation_covariance())  # Fixed: Slicing

    def update_state_and_covariance(self):
        self.x1 = self.x1_ + self.kalman_gain() @ self.y
        self.x0 = self.x1
        self.p1 = (np.eye(3) - self.kalman_gain() @ np.eye(len(self.y), 3)) @ self.p1_  # Fixed: Correct Kalman gain math
        self.p0 = self.p1