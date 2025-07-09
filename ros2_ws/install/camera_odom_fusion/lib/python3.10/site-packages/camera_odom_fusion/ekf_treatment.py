




# # projection_utils.py

# import numpy as np
# import yaml
# import math
# from pathlib import Path

# # — Paths to your existing YAMLs — no need to touch again
# BASE_TO_LIDAR_FILE = Path("./base2lidar_matrix.yaml")
# LIDAR_TO_CAMERA_FILE = Path("./Lidar2camera_matrix.yaml")
# INTRINSIC_MATRIX_FILE = Path(
#     "/media/zineddine/9D1D-BDBE1/"
#     "Turtlebot4_git/TurtleBot4/"
#     "Transformation_matrix/camera_intrinsics.yaml"
# )

# # — Load transforms & intrinsics once on import ——
# with BASE_TO_LIDAR_FILE.open() as f:
#     d = yaml.safe_load(f)
#     T_base_to_lidar = np.array(d["T_base_to_lidar"]["data"],dtype=np.float32).reshape(4,4)
# with LIDAR_TO_CAMERA_FILE.open() as f:
#     d = yaml.safe_load(f)
#     T_lidar_to_camera = np.array(d["extrinsic_matrix"],dtype=np.float32).reshape(4,4)
# with INTRINSIC_MATRIX_FILE.open() as f:
#     d = yaml.safe_load(f)
#     K = np.array(d["camera_matrix"]["data"],dtype=np.float32).reshape(3,3)
#     D = np.array(d["distortion_coefficients"]["data"],dtype=np.float32)

# # Combined Base→Camera
# T_base_to_camera = T_lidar_to_camera @ T_base_to_lidar

# # — Constants for your rectangles
# RECT_W_MM = 297.0
# RECT_H_MM = 210.0
# OBJ_PTS = np.array([
#     [0,          0,           0],
#     [RECT_W_MM,  0,           0],
#     [RECT_W_MM,  RECT_H_MM,   0],
#     [0,          RECT_H_MM,   0],
# ], dtype=np.float32)

# # — Helpers ———————————————————————————————————————————————

# def wrap_to_pi(angle: float) -> float:
#     return (angle + np.pi) % (2*np.pi) - np.pi

# def _pose_to_TWB(pose: np.ndarray) -> np.ndarray:
#     """From (x,y,theta) in world to 4×4 world→base frame."""
#     x,y,θ = pose
#     c,s = np.cos(θ), np.sin(θ)
#     return np.array([
#         [ c, -s, 0, x],
#         [ s,  c, 0, y],
#         [ 0,  0, 1, 0],
#         [ 0,  0, 0, 1],
#     ],dtype=np.float32)

# def project_world_to_camera(
#     world_pts: np.ndarray,
#     T_world_to_base: np.ndarray = np.eye(4)
# ) -> np.ndarray:
#     """
#     world_pts: (N,3) world coords
#     T_world_to_base: 4×4
#     returns: (N,3) camera coords
#     """
#     T = T_base_to_camera @ T_world_to_base
#     hom = np.hstack([world_pts, np.ones((len(world_pts),1),dtype=np.float32)])
#     cam = (T @ hom.T).T
#     return cam[:,:3]

# def project_on_image(
#     cam_pts: np.ndarray,
#     K: np.ndarray = K
# ) -> np.ndarray:
#     """
#     cam_pts: (N,3)
#     returns: (N,2) pixel coords
#     """
#     u = K[0,0]*cam_pts[:,0]/cam_pts[:,2] + K[0,2]
#     v = K[1,1]*cam_pts[:,1]/cam_pts[:,2] + K[1,2]
#     return np.stack([u,v],axis=1)

# def solve_pnp_from_bbox(
#     x: int, y: int, w: int, h: int,
#     K: np.ndarray = K, D: np.ndarray = D
# ):
#     """
#     4‐point solvePnP for your green rectangles.
#     """
#     img_pts = np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],dtype=np.float32)
#     ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, K, D,
#                                   flags=cv2.SOLVEPNP_ITERATIVE)
#     return ok, rvec, tvec, img_pts

# def jacobian_centre(
#     centre_W: np.ndarray,
#     T_world_to_base: np.ndarray,
#     x_pred: np.ndarray,
#     eps: float = 1e-5
# ) -> np.ndarray:
#     """
#     Numeric approx ∂(u,v)/∂(x,y,θ)
#     centre_W: (3,)
#     x_pred: (3,) [world→base pose]
#     """
#     T = _pose_to_TWB(x_pred)
#     P0 = project_world_to_camera(centre_W[None,:], T)[0]
#     uv0 = project_on_image(P0[None,:])[0]
#     J = np.zeros((2,3),dtype=np.float32)
#     if P0[2] <= 0:
#         return J
#     for i in range(3):
#         xp = x_pred.copy()
#         xp[i] += eps
#         Tp = _pose_to_TWB(xp)
#         Pp = project_world_to_camera(centre_W[None,:], Tp)[0]
#         uvp = project_on_image(Pp[None,:])[0]
#         J[:,i] = (uvp - uv0)/eps
#     return J




# def compute_bearing_and_jacobian(
#     x_pred: np.ndarray,
#     centre_W: np.ndarray
# ) -> tuple[float, np.ndarray]:
#     """
#     Calcule la direction prédite (z_pred) et son jacobien H (1×3)
#     pour un landmark donné.

#     Args:
#       x_pred   : état robot [x, y, θ]
#       centre_W : position du landmark en coordonnées monde [X, Y, Z]

#     Returns:
#       z_pred : angle (rad)
#       H      : (1×3) Jacobien ∂φ/∂[x, y, θ]
#     """
#     # projeté du landmark dans le repère caméra
#     T_WL = pose_to_TWL(x_pred)
#     P_C  = project_world_to_camera(centre_W[None, :], T_WL)[0]

#     # coordonnées image u
#     fx, cx0 = K[0,0], K[0,2]
#     u_c = project_on_image(P_C[None, :])[0, 0]
#     w   = (u_c - cx0) / fx

#     # z_pred : bearing prédite
#     z_pred = wrap_to_pi(math.atan(w))

#     # jacobien H (1×3)
#     J_uv = jacobian_centre(centre_W, T_WL, x_pred)  # 2×3
#     du_dx, du_dy, du_dtheta = J_uv[0]
#     dphi_du = 1.0 / (1.0 + w*w) / fx
#     H = np.array([[ 
#         dphi_du * du_dx,
#         dphi_du * du_dy,
#         dphi_du * du_dtheta
#     ]], dtype=np.float32)

#     return z_pred, H


    
# def choose(
#     x_pred: np.ndarray,
#     P_pred: np.ndarray,
#     mask: np.ndarray,
#     landmarks: dict[int,np.ndarray],
#     k: float = 2.0
# ) -> tuple[int,float,float]:
#     """
#     Your “enhanced probabilistic blob selection” exactly as before,
#     but now uses the unified project_world_to_camera/project_on_image.
#     """
#     T = _pose_to_TWB(x_pred)
#     best = (None, None, -np.inf)
#     for lid, centre in landmarks.items():
#         P_C = project_world_to_camera(centre[None,:], T)[0]
#         if P_C[2] <= 0: continue
#         u,v = project_on_image(P_C[None,:])[0]
#         fx,fy = K[0,0], K[1,1]

#         ℓ_u = fx * RECT_W_MM / P_C[2]
#         ℓ_v = fy * RECT_H_MM / P_C[2]
#         S_exp = ℓ_u*ℓ_v

#         J    = jacobian_centre(centre, T, x_pred)
#         Σ    = J @ P_pred @ J.T
#         σ_u  = k*np.sqrt(Σ[0,0])
#         σ_v  = k*np.sqrt(Σ[1,1])
#         S_prob = (2*σ_u)*(2*σ_v)

#         C1 = S_exp/S_prob
#         Npix = np.sum(mask==lid)
#         C2 = Npix/S_exp

#         score = C1*C2
#         if score>best[2]:
#             best = (lid,
#                     wrap_to_pi(math.atan((u-K[0,2])/fx)),
#                     score)
#     return best

# def project_rectangle_corners(
#     centre_W: np.ndarray,
#     x_pred: np.ndarray
# ) -> np.ndarray:
#     """
#     Returns (4,2) pixel‐coords of your rectangle corners,
#     *exactly* like your first file did if you replace OBJ_PTS.
#     """
#     # build world‐coords of 4 corners
#     offs = OBJ_PTS  # as defined above
#     world_corners = centre_W[None,:] + offs
#     cam = project_world_to_camera(world_corners, _pose_to_TWB(x_pred))
#     return project_on_image(cam)

# def get_uncertainty_box(
#     centre_W: np.ndarray,
#     x_pred: np.ndarray,
#     P_pred: np.ndarray,
#     k: float = 2.0
# ) -> tuple[int,int,int,int]:
#     """
#     Exactly your k‐sigma expansion box.
#     """
#     # reuse part of choose() to get σ_u,σ_v
#     T = _pose_to_TWB(x_pred)
#     P_C = project_world_to_camera(centre_W[None,:], T)[0]
#     if P_C[2] <= 0:
#         return None
#     u,v = project_on_image(P_C[None,:])[0]
#     J   = jacobian_centre(centre_W, T, x_pred)
#     Σ   = J @ P_pred @ J.T
#     σ_u = k*np.sqrt(Σ[0,0]);  σ_v = k*np.sqrt(Σ[1,1])

#     ℓ_u = K[0,0]*RECT_W_MM/P_C[2]
#     ℓ_v = K[1,1]*RECT_H_MM/P_C[2]
#     w = int(ℓ_u + 2*σ_u);  h = int(ℓ_v + 2*σ_v)
#     x = int(u - w/2);      y = int(v - h/2)
#     return x,y,w,h

# def bearing_from_bbox(
#     cx: float,
#     img_w: int
# ) -> float:
#     """Same as your old function."""
#     return math.degrees(math.atan2(cx - img_w/2, K[0,0]))
#@@@@@@@@@@@@@@
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math
from rclpy.qos   import qos_profile_sensor_data
import json
import csv
import os

class EKF_estimation(Node):
    def __init__(self):
        super().__init__('ekf_estimation')
        ## this part is for logs ####

        # Add after your other __init__ code
        self.log_dir = os.path.expanduser('~/ekf_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        # Step counters
        self.meas_step = 0
        self.state_step = 0

        # Measurement log: step, z_pred, z_meas, landmark_id
        self.meas_log_path = os.path.join(self.log_dir, 'meas_log.csv')
        self.meas_log_file = open(self.meas_log_path, 'w', newline='')
        self.meas_log_writer = csv.writer(self.meas_log_file)
        self.meas_log_writer.writerow(['step', 'z_pred', 'z_meas', 'landmark_id'])

        # State log: step, x_pred, y_pred, theta_pred, x_updt, y_updt, theta_updt
        self.state_log_path = os.path.join(self.log_dir, 'state_log.csv')
        self.state_log_file = open(self.state_log_path, 'w', newline='')
        self.state_log_writer = csv.writer(self.state_log_file)
        self.state_log_writer.writerow(['step', 'x_pred', 'y_pred', 'theta_pred', 'x_updt', 'y_updt', 'theta_updt'])
        ##
        # Initial robot state: x_t = [xt, yt, thetat]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # Initial covariance
        self.P_est = np.diag([0.5, 0.5, np.deg2rad(10)])**2

        # Process noise (odometry uncertainty)
        # Represents uncertainty in U = [xu, yu, thetau] measurements
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])**2

        # Measurement noise (camera measurements z_k = [bearing])
        self.R = np.array([[np.deg2rad(5)**2]]) 

        # Store z_h data from topic
        self.z_meas = None
        self.z_pred = None
        self.H = None
        self.landmark_id = None

        # Store previous odometry for calculating deltas
        self.prev_odom = None

        # Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/get_odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.x_pred_publisher = self.create_publisher(String, '/get_xpred', 10)
        
        # Subscribers
        self.z_h_subscriber = self.create_subscription(
            String,
            '/get_z_h',
            self.z_h_callback,
            10
        )

        self.get_logger().info('EKF Estimation Node initialized')

    def z_h_callback(self, msg):
        # Process z_h data
        try:
            z_h_data = json.loads(msg.data)
            
            # Handle None values explicitly
            if z_h_data.get("z_meas") is None:
                self.z_meas = None
                self.z_pred = None
                self.H = None
                self.landmark_id = None
                self.get_logger().debug("Received null z_h data")
            else:
                self.z_meas = float(z_h_data.get("z_meas"))
                self.z_pred = float(z_h_data.get("z_pred"))
                self.H = np.array(z_h_data.get("H"), dtype=np.float32)
                self.landmark_id = int(z_h_data.get("landmark_id"))
                
                self.get_logger().info(
                    f"Received z_h: meas={self.z_meas:.3f}, pred={self.z_pred:.3f}, "
                    f"landmark={self.landmark_id}"
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in z_h_callback: {str(e)}')

    def odom_callback(self, msg):
        """
        Process odometry and run EKF
        Extract current pose and calculate U = [xu, yu, thetau] from odometry evolution
        """
        try:
            # Extract current position and orientation
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            
            # Convert quaternion to euler angle (yaw)
            orientation_q = msg.pose.pose.orientation
            current_theta = self.quaternion_to_euler(orientation_q)
            
            # Calculate odometry deltas if we have previous data
            if self.prev_odom is not None:
                # Calculate displacement in global frame
                dx_global = current_x - self.prev_odom[0]
                dy_global = current_y - self.prev_odom[1]
                dtheta = current_theta - self.prev_odom[2]
                
                # Wrap angle difference
                dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
                
                # Transform global displacement to robot frame at previous position
                cos_prev_theta = math.cos(self.prev_odom[2])
                sin_prev_theta = math.sin(self.prev_odom[2])
                
                # U = [xu, yu, thetau] in robot frame
                xu = dx_global * cos_prev_theta + dy_global * sin_prev_theta
                yu = -dx_global * sin_prev_theta + dy_global * cos_prev_theta
                thetau = dtheta
                
                U = np.array([xu, yu, thetau])
                
                # Only run EKF if we have significant motion to avoid numerical issues
                if np.linalg.norm(U) > 1e-6:
                    self.run_ekf_step(U)
                else:
                    # Still publish current prediction for camera node
                    self.publish_x_pred()
            else:
                # First odometry message, just publish initial state
                self.publish_x_pred()
            
            # Store current odometry for next iteration
            self.prev_odom = np.array([current_x, current_y, current_theta])
            
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def quaternion_to_euler(self, q):
        """
        Convert quaternion to euler angle (yaw only)
        """
        # Calculate yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def run_ekf_step(self, U):
        """
        Complete EKF step (predict + update)
        
        Parameters:
            U: odometry command [xu, yu, thetau]
        
        Returns:
            x_est: updated state estimate
            P_est: updated covariance estimate
        """
        # Prediction step
        x_pred, P_pred = self.predict_ekf(self.x_est, self.P_est, U)
        
        # Update step (if measurement is available from z_h topic)
        if self.z_meas is not None and self.z_pred is not None and self.H is not None:
            self.x_est, self.P_est = self.update_ekf(x_pred, P_pred, self.R)
            self.get_logger().info(
                f"EKF Update: x=[{self.x_est[0]:.3f}, {self.x_est[1]:.3f}, {np.degrees(self.x_est[2]):.1f}°]"
            )
            self.meas_log_writer.writerow([
            self.meas_step,
            self.z_pred,
            self.z_meas,
            self.landmark_id
            ])
            self.meas_log_file.flush()
            self.meas_step += 1

        else:
            # No measurement available, use prediction only
            self.x_est = x_pred
            self.P_est = P_pred
            self.get_logger().debug("EKF Prediction only (no measurement)")
        
        self.state_log_writer.writerow([
        self.state_step,
            x_pred[0], x_pred[1], x_pred[2],  # prediction
            self.x_est[0], self.x_est[1], self.x_est[2]  # estimated (updated) state
        ])
        self.state_log_file.flush()
        self.state_step += 1


        return self.x_est, self.P_est

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
        
        # Publish x_pred
        self.publish_x_pred_data(x_pred, P_pred)
        
        return x_pred, P_pred

    def publish_x_pred_data(self, x_pred, P_pred):
        """Helper function to publish x_pred data"""
        try:
            x_pred_data = {
                "x_pred": x_pred.tolist(), 
                "P_pred": P_pred.tolist(),
                "timestamp": self.get_clock().now().to_msg()._sec + self.get_clock().now().to_msg()._nanosec * 1e-9
            }
            x_pred_msg = String()
            x_pred_msg.data = json.dumps(x_pred_data)
            self.x_pred_publisher.publish(x_pred_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing x_pred: {str(e)}')

    def publish_x_pred(self):
        """Publish current state estimate"""
        self.publish_x_pred_data(self.x_est, self.P_est)

    def update_ekf(self, x_pred, P_pred, R):
        """
        Update step using camera measurements from z_h topic
        
        Parameters:
            x_pred: predicted state
            P_pred: predicted covariance
            R: measurement noise covariance (1x1 matrix for bearing)
        
        Returns:
            x_upd: updated state
            P_upd: updated covariance
        """
        # Use values from z_h topic
        if self.z_meas is None or self.z_pred is None or self.H is None:
            return x_pred, P_pred
        
        z_k = self.z_meas
        z_pred = self.z_pred
        print(f"z_meas = {z_k},         z_pred = {z_pred}")
        H = self.H
        
        # Ensure H is the correct shape (1x3)
        if H.shape != (1, 3):
            self.get_logger().error(f"H has wrong shape: {H.shape}, expected (1, 3)")
            return x_pred, P_pred
        
        # Innovation
        y = np.array([z_k - z_pred])
        y[0] = (y[0] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Check for numerical issues
        if np.linalg.det(S) < 1e-10:
            self.get_logger().warn("Innovation covariance S is nearly singular")
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Update state
        x_upd = x_pred + K @ y
        x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ H
        P_upd = I_KH @ P_pred @ I_KH.T + K @ R @ K.T
        
        # Log innovation for debugging
        self.get_logger().debug(
            f"Innovation: y={np.degrees(y[0]):.1f}°, "
            f"S={np.sqrt(S[0,0]):.3f}, "
            f"K_norm={np.linalg.norm(K):.3f}"
        )
        
        return x_upd, P_upd

    def get_state_estimate(self):
        """Return current state estimate x_t = [xt, yt, thetat]"""
        return self.x_est.copy()
    
    def get_covariance_estimate(self):
        """Return current covariance estimate"""
        return self.P_est.copy()

    def __del__(self):
        self.meas_log_file.close()
        self.state_log_file.close()



def main(args=None):
    rclpy.init(args=args)
    
    ekf_node = EKF_estimation()
    
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()