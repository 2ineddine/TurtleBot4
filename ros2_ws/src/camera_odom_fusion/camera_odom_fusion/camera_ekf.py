# import numpy as np
# import math

# # ---------------------------------------------------------------------
# # Camera / landmark observation model for an EKF
# #
# # Measurement: a single bearing angle ϕ_k (rad) from the robot’s
# #               optical axis to a known landmark.
# #
# # Robot state  x  = [x, y, θ]ᵀ            (world frame)
# # Landmark     l  = [x_L, y_L]            (world frame, known)
# #
# # Predicted bearing
# #   ϕ̂(x,l) = wrap_to_pi( atan2(y_L - y, x_L - x) - θ )
# #
# # Jacobian H = ∂ϕ̂/∂x evaluated at the predicted state
# # ---------------------------------------------------------------------

# def wrap_to_pi(angle):
#     """Wrap any angle to the interval [-π, π]."""
#     return (angle + np.pi) % (2 * np.pi) - np.pi


# def bearing_only_model(x, landmark):
#     """
#     Compute the predicted bearing and its Jacobian.

#     Parameters
#     ----------
#     x : ndarray, shape (3,)
#         Robot state [x, y, θ].
#     landmark : ndarray, shape (2,)
#         Landmark position [x_L, y_L].

#     Returns
#     -------
#     phi_hat : float
#         Predicted bearing angle (rad) robot → landmark.
#     H : ndarray, shape (1, 3)
#         Jacobian ∂ϕ̂/∂x evaluated at x.
#     """
#     dx = landmark[0] - x[0]
#     dy = landmark[1] - x[1]
#     r2 = dx * dx + dy * dy  # squared range

#     # Predicted bearing (measurement expectation)
#     phi_hat = wrap_to_pi(math.atan2(dy, dx) - x[2])

#     # Jacobian – valid unless robot exactly on landmark
#     if r2 < 1e-12:          # avoid division by zero
#         H = np.array([[0.0, 0.0, -1.0]])
#     else:
#         H = np.array([[dy / r2, -dx / r2, -1.0]])

#     return phi_hat, H


# def ekf_bearing_update(x_pred, P_pred, phi_meas, landmark,
#                        R_scalar=np.deg2rad(5.0)**2):
#     """
#     EKF update step for a single bearing measurement.

#     Parameters
#     ----------
#     x_pred : ndarray, shape (3,)
#         Predicted state from the motion model.
#     P_pred : ndarray, shape (3, 3)
#         Predicted covariance.
#     phi_meas : float
#         Bearing measurement (rad).
#     landmark : ndarray, shape (2,)
#         Landmark position [x_L, y_L].
#     R_scalar : float, optional
#         Bearing measurement variance (σ_ϕ²).

#     Returns
#     -------
#     x_upd : ndarray, shape (3,)
#         Updated state estimate.
#     P_upd : ndarray, shape (3, 3)
#         Updated covariance.
#     innovation : float
#         Bearing innovation (meas − pred, wrapped to [-π, π]).
#     """
#     # 1. Expected measurement & Jacobian
#     phi_hat, H = bearing_only_model(x_pred, landmark)

#     # 2. Innovation (measurement residual)
#     innovation = wrap_to_pi(phi_meas - phi_hat)

#     # 3. Innovation covariance
#     S = H @ P_pred @ H.T + R_scalar          # scalar: 1×1

#     # 4. Kalman gain
#     K = (P_pred @ H.T) / S                   # (3×1)

#     # 5. State update
#     x_upd = x_pred + (K * innovation).ravel()
#     x_upd[2] = wrap_to_pi(x_upd[2])          # keep θ inside [-π, π]

#     # 6. Covariance update (Joseph form optional)
#     P_upd = (np.eye(3) - K @ H) @ P_pred

#     return x_upd, P_upd, innovation

#@@@@@@@@@@@@@@@@@@@@@@@@@








# # # projection_utils.py

# import numpy as np
# import yaml
# import math
# from pathlib import Path


# import yaml
# import os
# from ament_index_python.packages import get_package_share_directory
# import cv2
# # Locate the package share directory
# package_share = get_package_share_directory('camera_odom_fusion')

# # Define YAML paths inside your ROS 2 package
# BASE_TO_LIDAR_FILE = os.path.join(package_share, 'base2lidar_matrix.yaml')
# LIDAR_TO_CAMERA_FILE = os.path.join(package_share, 'Lidar2camera_matrix.yaml')

# # Optional: make this a parameter, or also load from share dir
# INTRINSIC_MATRIX_FILE = "/media/zineddine/9D1D-BDBE1/Turtlebot4_git/TurtleBot4/Transformation_matrix/camera_intrinsics.yaml"

# # Helper function to load and reshape matrices from YAML
# def load_matrix(file_path, key, shape):
#     with open(file_path, 'r') as f:
#         data = yaml.safe_load(f)
#         return np.array(data[key]["data"], dtype=np.float32).reshape(shape)

# # Load all matrices
# T_base_to_lidar = load_matrix(BASE_TO_LIDAR_FILE, "T_base_to_lidar", (4, 4))
# # T_lidar_to_camera = load_matrix(LIDAR_TO_CAMERA_FILE, "extrinsic_matrix", (4, 4))
# with open(LIDAR_TO_CAMERA_FILE, 'r') as f:
#     d = yaml.safe_load(f)
#     T_lidar_to_camera = np.array(d["extrinsic_matrix"], dtype=np.float32)

# with open(INTRINSIC_MATRIX_FILE, 'r') as f:
#     d = yaml.safe_load(f)
#     K = np.array(d["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
#     D = np.array(d["distortion_coefficients"]["data"], dtype=np.float32)


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
#@@@@@@@@@@@@@@@@
import numpy as np
import yaml
import math
from pathlib import Path
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import cv2

# Locate the package share directory
package_share = get_package_share_directory('camera_odom_fusion')

# Define YAML paths inside your ROS 2 package
BASE_TO_LIDAR_FILE = os.path.join(package_share, 'base2lidar_matrix.yaml')
LIDAR_TO_CAMERA_FILE = os.path.join(package_share, 'Lidar2camera_matrix.yaml')

# Optional: make this a parameter, or also load from share dir
INTRINSIC_MATRIX_FILE = "/media/zineddine/9D1D-BDBE1/Turtlebot4_git/TurtleBot4/Transformation_matrix/camera_intrinsics.yaml"

# Helper function to load and reshape matrices from YAML
def load_matrix(file_path, key, shape):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        return np.array(data[key]["data"], dtype=np.float32).reshape(shape)

# Load all matrices
T_base_to_lidar = load_matrix(BASE_TO_LIDAR_FILE, "T_base_to_lidar", (4, 4))
# T_lidar_to_camera = load_matrix(LIDAR_TO_CAMERA_FILE, "extrinsic_matrix", (4, 4))
with open(LIDAR_TO_CAMERA_FILE, 'r') as f:
    d = yaml.safe_load(f)
    T_lidar_to_camera = np.array(d["extrinsic_matrix"], dtype=np.float32)

with open(INTRINSIC_MATRIX_FILE, 'r') as f:
    d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], dtype=np.float32)

# Combined Base→Camera
T_base_to_camera = T_lidar_to_camera @ T_base_to_lidar

# — Constants for your rectangles (in meters)
RECT_W_MM = 0.297  # 297mm = 0.297m
RECT_H_MM = 0.210  # 210mm = 0.210m

# Replace the OBJ_PTS definition with:
half_width = RECT_W_MM / 2.0
half_height = RECT_H_MM / 2.0

OBJ_PTS = np.array([
    [-half_width, -half_height, 0],    # Bottom-left of center
    [+half_width, -half_height, 0],    # Bottom-right of center
    [+half_width, +half_height, 0],    # Top-right of center
    [-half_width, +half_height, 0],    # Top-left of center
], dtype=np.float32)

# — Helpers ———————————————————————————————————————————————
def pose_to_TWL(pose: np.ndarray) -> np.ndarray:
    """From (x,y,theta) in world to 4×4 world→base frame."""
    x, y, θ = pose
    c, s = np.cos(θ), np.sin(θ)
    return np.array([
        [ c, -s, 0, x],
        [ s,  c, 0, y],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1],
    ], dtype=np.float32)

def wrap_to_pi(angle: float) -> float:
    return (angle + np.pi) % (2*np.pi) - np.pi

def _pose_to_TWB(pose: np.ndarray) -> np.ndarray:
    """From (x,y,theta) in world to 4×4 world→base frame."""
    x,y,θ = pose
    c,s = np.cos(θ), np.sin(θ)
    return np.array([
        [ c, -s, 0, x],
        [ s,  c, 0, y],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1],
    ],dtype=np.float32)

def project_world_to_camera(world_pts, T_world_to_base):
    # Convert to homogeneous coordinates (N,3) -> (N,4)
    hom_pts = np.hstack([world_pts, np.ones((world_pts.shape[0], 1))])
    
    # Apply transformations: T_camera^base @ T_base^world @ pts_world
    cam_pts = (T_base_to_camera @ T_world_to_base @ hom_pts.T).T
    
    # Return only valid points (Z > 0)
    valid = cam_pts[:, 2] > 0
    return cam_pts[valid, :3]  # Strip homogeneous coord

def project_on_image(cam_pts, K=K, image_width=640, image_height=480):
    if len(cam_pts) == 0:
        return np.array([]).reshape(0, 2)
    
    u = (K[0, 0] * cam_pts[:, 0] / cam_pts[:, 2]) + K[0, 2]
    v = (K[1, 1] * cam_pts[:, 1] / cam_pts[:, 2]) + K[1, 2]
    # Check if projected points are within image bounds
    valid = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)
    return np.stack([u[valid], v[valid]], axis=1)  # Only return visible points

def solve_pnp_from_bbox(
    x: int, y: int, w: int, h: int,
    K: np.ndarray = K, D: np.ndarray = D
):
    """
    4‐point solvePnP for your green rectangles.
    """
    img_pts = np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, K, D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    return ok, rvec, tvec, img_pts

def jacobian_centre(
    centre_W: np.ndarray,
    T_world_to_base: np.ndarray,
    x_pred: np.ndarray,
    eps: float = 1e-5
) -> np.ndarray:
    """
    Numeric approx ∂(u,v)/∂(x,y,θ)
    centre_W: (3,)
    x_pred: (3,) [world→base pose]
    """
    T = _pose_to_TWB(x_pred)
    P0_arr = project_world_to_camera(centre_W[None,:], T)
    if len(P0_arr) == 0:
        return np.zeros((2,3), dtype=np.float32)
    P0 = P0_arr[0]
    
    uv0_arr = project_on_image(P0[None,:])
    if len(uv0_arr) == 0:
        return np.zeros((2,3), dtype=np.float32)
    uv0 = uv0_arr[0]
    
    J = np.zeros((2,3),dtype=np.float32)
    if P0[2] <= 0:
        return J
    for i in range(3):
        xp = x_pred.copy()
        xp[i] += eps
        Tp = _pose_to_TWB(xp)
        Pp_arr = project_world_to_camera(centre_W[None,:], Tp)
        if len(Pp_arr) == 0:
            continue
        Pp = Pp_arr[0]
        uvp_arr = project_on_image(Pp[None,:])
        if len(uvp_arr) == 0:
            continue
        uvp = uvp_arr[0]
        J[:,i] = (uvp - uv0)/eps
    return J

def compute_bearing_and_jacobian(
    x_pred: np.ndarray,
    centre_W: np.ndarray
) -> tuple[float, np.ndarray]:
    """
    Calcule la direction prédite (z_pred) et son jacobien H (1×3)
    pour un landmark donné.

    Args:
      x_pred   : état robot [x, y, θ]
      centre_W : position du landmark en coordonnées monde [X, Y, Z]

    Returns:
      z_pred : angle (rad)
      H      : (1×3) Jacobien ∂φ/∂[x, y, θ]
    """
    # projeté du landmark dans le repère caméra
    T_WL = pose_to_TWL(x_pred)
    P_C_arr = project_world_to_camera(centre_W[None, :], T_WL)
    if len(P_C_arr) == 0:
        return 0.0, np.zeros((1,3), dtype=np.float32)
    P_C = P_C_arr[0]

    # coordonnées image u
    fx, cx0 = K[0,0], K[0,2]
    uv_arr = project_on_image(P_C[None, :])
    if len(uv_arr) == 0:
        return 0.0, np.zeros((1,3), dtype=np.float32)
    u_c = uv_arr[0, 0]
    w   = (u_c - cx0) / fx

    # z_pred : bearing prédite
    z_pred = wrap_to_pi(math.atan(w))

    # jacobien H (1×3)
    J_uv = jacobian_centre(centre_W, T_WL, x_pred)  # 2×3
    du_dx, du_dy, du_dtheta = J_uv[0]
    dphi_du = 1.0 / (1.0 + w*w) / fx
    H = np.array([[ 
        dphi_du * du_dx,
        dphi_du * du_dy,
        dphi_du * du_dtheta
    ]], dtype=np.float32)

    return z_pred, H

def choose(
    x_pred: np.ndarray,
    P_pred: np.ndarray,
    mask: np.ndarray,
    landmarks: dict[int,np.ndarray],
    k: float = 2.0
) -> tuple[int,float,float]:
    """
    Enhanced probabilistic blob selection with proper error handling.
    """
    T = pose_to_TWL(x_pred)
    best = (None, None, -np.inf)
    
    for lid, centre in landmarks.items():
        try:
            # Check if landmark is visible in camera
            P_C_arr = project_world_to_camera(centre[None,:], T)
            if len(P_C_arr) == 0:
                continue  # Landmark is behind camera
            P_C = P_C_arr[0]
            
            if P_C[2] <= 0:
                continue  # Behind camera
                
            # Project to image coordinates
            uv_arr = project_on_image(P_C[None,:])
            if len(uv_arr) == 0:
                continue  # Outside image bounds
            u, v = uv_arr[0]
            
            fx, fy = K[0,0], K[1,1]

            # Calculate expected size
            l_u = fx * RECT_W_MM / P_C[2]
            l_v = fy * RECT_H_MM / P_C[2]
            S_exp = l_u * l_v

            # Calculate uncertainty
            J = jacobian_centre(centre, T, x_pred)
            if J.size == 0:
                continue
            Σ = J @ P_pred @ J.T
            ro_u = k * np.sqrt(max(0, Σ[0,0]))  # Ensure non-negative
            ro_v = k * np.sqrt(max(0, Σ[1,1]))  # Ensure non-negative
            S_prob = (2*ro_u) * (2*ro_v)

            if S_prob <= 0:
                continue

            C1 = S_exp / S_prob
            Npix = np.sum(mask == lid)
            
            if S_exp <= 0:
                continue
            C2 = Npix / S_exp

            score = C1 * C2
            if score > best[2]:
                bearing = wrap_to_pi(math.atan((u - K[0,2]) / fx))
                best = (lid, bearing, score)
                
        except Exception as e:
            # Log error but continue processing other landmarks
            print(f"Error processing landmark {lid}: {e}")
            continue
    
    return best

def project_rectangle_corners(centre_W, x_pred, img_w, img_h):
    """
    Project rectangle corners with proper error handling.
    """
    try:
        # World coordinates of the 4 rectangle corners
        corners_3D = centre_W[None, :] + OBJ_PTS
        
        # Transform to camera coordinates
        cam_corners = project_world_to_camera(corners_3D, pose_to_TWL(x_pred))
        if len(cam_corners) < 4:  # At least one corner is behind the camera
            return None, False
        
        # Project to 2D image coordinates
        img_corners = project_on_image(cam_corners, K, img_w, img_h)
        if len(img_corners) < 4:  # At least one corner is outside the image
            return None, False
        
        return img_corners, True  # All corners are valid
    except Exception as e:
        print(f"Error in project_rectangle_corners: {e}")
        return None, False

def get_uncertainty_box(
    centre_W: np.ndarray,
    x_pred: np.ndarray,
    P_pred: np.ndarray,
    k: float = 0.5
) -> tuple[int,int,int,int]:
    """
    Calculate uncertainty box with proper error handling.
    """
    try:
        T = pose_to_TWL(x_pred)
        P_C_arr = project_world_to_camera(centre_W[None,:], T)
        if len(P_C_arr) == 0:
            return None
        P_C = P_C_arr[0]
        
        if P_C[2] <= 0:
            return None
            
        uv_arr = project_on_image(P_C[None,:])
        if len(uv_arr) == 0:
            return None
        u, v = uv_arr[0]
        
        J = jacobian_centre(centre_W, T, x_pred)
        if J.size == 0:
            return None
        Σ = J @ P_pred @ J.T
        ro_u = k * np.sqrt(max(0, Σ[0,0]))
        ro_v = k * np.sqrt(max(0, Σ[1,1]))

        l_u = K[0,0] * RECT_W_MM / P_C[2]
        l_v = K[1,1] * RECT_H_MM / P_C[2]
        w = int(l_u + 2*ro_u)
        h = int(l_v + 2*ro_v)
        x = int(u - w/2)
        y = int(v - h/2)
        return x, y, w, h
    except Exception as e:
        print(f"Error in get_uncertainty_box: {e}")
        return None

def bearing_from_bbox(
    cx: float,
    img_w: int
) -> float:
    """Same as your old function."""
    return math.degrees(math.atan2(cx - img_w/2, K[0,0]))