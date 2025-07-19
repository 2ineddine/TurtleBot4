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
INTRINSIC_MATRIX_FILE = "/media/zineddine/9D1D-BDBE1/Turtlebot4_git/TurtleBot4/camera_intrinsics_250x250.yaml"

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
RECT_W_MM = 0.35  
RECT_H_MM = 0.25 

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

def pose_to_T_matrix(pose: np.ndarray) -> np.ndarray:
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


def project_world_to_camera_frame(world_pts, T_world_to_base):
    # Convert to homogeneous coordinates (N,3) -> (N,4)
    hom_pts = np.hstack([world_pts, np.ones((world_pts.shape[0], 1))])
    
    # Apply transformations: T_camera^base @ T_base^world @ pts_world
    cam_pts = (T_base_to_camera @ T_world_to_base @ hom_pts.T).T
    
    # Return only valid points (Z > 0)
    valid = cam_pts[:, 2] > 0
    return cam_pts[valid, :3]  # Strip homogeneous coord

def from_3D_to_2d_camera(cam_pts, K=K, image_width=250, image_height=250):
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
    T = pose_to_T_matrix(x_pred)
    P0_arr = project_world_to_camera_frame(centre_W[None,:], T)
    if len(P0_arr) == 0:
        return np.zeros((2,3), dtype=np.float32)
    P0 = P0_arr[0]
    
    # FIX: Make consistent - add missing parameters
    uv0_arr = from_3D_to_2d_camera(P0[None,:], K, 250, 250)
    if len(uv0_arr) == 0:
        return np.zeros((2,3), dtype=np.float32)
    uv0 = uv0_arr[0]
    
    J = np.zeros((2,3),dtype=np.float32)
    if P0[2] <= 0:
        return J
    for i in range(3):
        xp = x_pred.copy()
        xp[i] += eps
        Tp = pose_to_T_matrix(xp)
        Pp_arr = project_world_to_camera_frame(centre_W[None,:], Tp)
        if len(Pp_arr) == 0:
            continue
        Pp = Pp_arr[0]
        # Already correct
        uvp_arr = from_3D_to_2d_camera(Pp[None,:], K, 250, 250)
        if len(uvp_arr) == 0:
            continue
        uvp = uvp_arr[0]
        J[:,i] = (uvp - uv0)/eps
    return J

def chose_landmark(
    x_pred: np.ndarray,
    P_pred: np.ndarray,
    mask: np.ndarray,
    landmarks: dict[int,np.ndarray],
    k: float = 1.0
) -> tuple[int,float,float]:
    """
    Enhanced probabilistic blob selection with proper error handling.
    """
    T = pose_to_T_matrix(x_pred)
    best = (None, None, -np.inf)
    
    for lid, centre in landmarks.items():
        try:
            # Check if landmark is visible in camera
            P_C_arr = project_world_to_camera_frame(centre[None,:], T)
            if len(P_C_arr) == 0:
                continue  # Landmark is behind camera
            P_C = P_C_arr[0]
            
            if P_C[2] <= 0:
                continue  # Behind camera
                
            # Project to image coordinates
            uv_arr = from_3D_to_2d_camera(P_C[None,:],K, 250, 250)
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
            print (np.unique(mask))
            C1 = S_exp / S_prob
            Npix = np.sum(mask == lid)
            
            if S_exp <= 0:
                continue
            C2 = Npix / S_exp

            score = C1 * C2
            print (f"score  : {score} , Npix : {Npix}, C2 : {C2}, C1 : {C1}  ")
            if score > best[2]:
                bearing = wrap_to_pi(math.atan((u - K[0,2]) / fx))
                #print ("u = ",u,"K  = ",K[0,2],"fx = ", fx)
                best = (lid, bearing, score)

                
        except Exception as e:
            # Log error but continue processing other landmarks
            print(f"Error processing landmark {lid}: {e}")
            continue

        #print ("bearing : ",bearing,"and its size is ",score.shape )
    
    return best


def get_uncertainty_box(
    centre_W: np.ndarray,
    x_pred: np.ndarray,
    P_pred: np.ndarray,
    img_w,img_h,
    k: float = 0.5
) -> tuple[np.ndarray, np.ndarray, bool, bool]:
    """
    Calculate both blue and red rectangle corners.
    Returns: blue_corners, red_corners, blue_valid, red_valid
    """
    try:
        # Blue rectangle (nominal pose)
        blue_corners, blue_valid = project_rectangle_corners(centre_W, x_pred, img_w, img_h)
        
        # Red rectangle (uncertain pose)
        sigma_x = np.sqrt(max(0, P_pred[0,0]))
        sigma_y = np.sqrt(max(0, P_pred[1,1]))
        sigma_theta = np.sqrt(max(0, P_pred[2,2]))
        
        x_uncertain = x_pred + k * np.array([sigma_x, sigma_y, sigma_theta])
        red_corners, red_valid = project_rectangle_corners(centre_W, x_uncertain, img_w, img_h)
        
        return blue_corners, red_corners, blue_valid, red_valid
        
    except Exception as e:
        print(f"Error in get_uncertainty_box: {e}")
        return None, None, False, False









def project_rectangle_corners(centre_W, x_pred, img_w, img_h):
    """
    Use the same transformation as compute_bearing_and_jacobian
    """
    try:
        # Use consistent transformation - same as compute_bearing_and_jacobian
        T_WL = pose_to_T_matrix(x_pred)
        
        # Get all 4 corners of the rectangle
        corners_3D = centre_W[None, :] + OBJ_PTS  # (4, 3)
        
        # Project all corners to camera coordinates
        camera_corners = project_world_to_camera_frame(corners_3D, T_WL)
        
        if len(camera_corners) < 4:
            return None, False
            
        # FIX: Correct parameter passing - remove duplicates
        img_corners = from_3D_to_2d_camera(camera_corners, K, img_w, img_h)
        
        if len(img_corners) < 4:
            return None, False
            
        return img_corners, True
        
    except Exception as e:
        return None, False
























def compute_bearing_and_jacobian(
    x_pred: np.ndarray,
    centre_W: np.ndarray
) -> tuple[float, np.ndarray]:
    """
    Calculates bearing angle and its Jacobian for angle-only measurements.
    This version computes bearing in world frame and derives H matrix analytically.

    Args:
      x_pred   : robot state [x, y, θ] in world frame
      centre_W : landmark position [X, Y, Z] in world frame

    Returns:
      z_pred : bearing angle (rad) - angle from robot heading to landmark
      H      : (1×3) Jacobian ∂bearing/∂[x, y, θ]
    """
    
    # Extract robot pose components
    x_robot, y_robot, theta_robot = x_pred
    
    # Extract landmark position (only x,y needed for bearing)
    x_landmark, y_landmark = centre_W[0], centre_W[1]
    
    # Calculate relative position vector (world frame)
    Delta_x = x_landmark - x_robot  # Δx
    Delta_y = y_landmark - y_robot  # Δy
    
    # Calculate squared distance
    r_squared = Delta_x**2 + Delta_y**2
    
    # Check if landmark is too close (avoid division by zero)
    if r_squared < 1e-6:
        return 0.0, np.zeros((1, 3), dtype=np.float32)
    
    # Calculate world-frame angle from robot to landmark
    world_angle = math.atan2(Delta_y, Delta_x)
    
    # Calculate bearing: angle from robot heading to landmark
    z_pred = world_angle - theta_robot
    
    # Normalize bearing to [-π, π]
    z_pred = wrap_to_pi(z_pred)
    
    # Calculate H matrix analytically
    # H = [∂bearing/∂x, ∂bearing/∂y, ∂bearing/∂θ]
    
    # ∂bearing/∂x = ∂/∂x[atan2(Δy, Δx) - θ] = ∂/∂x[atan2(Δy, Δx)]
    # Since Δx = x_landmark - x_robot, ∂Δx/∂x = -1
    # ∂atan2(Δy, Δx)/∂x = -Δy/r² * ∂Δx/∂x = -Δy/r² * (-1) = Δy/r²
    dh_dx = Delta_y / r_squared
    
    # ∂bearing/∂y = ∂/∂y[atan2(Δy, Δx) - θ] = ∂/∂y[atan2(Δy, Δx)]
    # Since Δy = y_landmark - y_robot, ∂Δy/∂y = -1  
    # ∂atan2(Δy, Δx)/∂y = Δx/r² * ∂Δy/∂y = Δx/r² * (-1) = -Δx/r²
    dh_dy = -Delta_x / r_squared
    
    # ∂bearing/∂θ = ∂/∂θ[atan2(Δy, Δx) - θ] = -1
    dh_dtheta = -1.0
    
    # Construct H matrix
    H = np.array([[dh_dx, dh_dy, dh_dtheta]], dtype=np.float32)
    print ("z_pred ", z_pred,"H = ",H)
    
    return z_pred, H


def compute_bearing_and_jacobian2(
    x_pred: np.ndarray,
    centre_W: np.ndarray
) -> tuple[float, np.ndarray]:
    """
    Alternative version that uses camera projection for bearing calculation.
    This maintains compatibility with camera-based systems while computing angle-only measurements.

    Args:
      x_pred   : robot state [x, y, θ] in world frame  
      centre_W : landmark position [X, Y, Z] in world frame

    Returns:
      z_pred : bearing angle (rad)
      H      : (1×3) Jacobian ∂bearing/∂[x, y, θ]
    """
    
    # Project landmark to camera coordinates
    T_WL = pose_to_TWL(x_pred)  # Using existing function
    P_C_arr = project_world_to_camera(centre_W[None, :], T_WL)
    if len(P_C_arr) == 0:
        return 0.0, np.zeros((1, 3), dtype=np.float32)
    P_C = P_C_arr[0]
    
    if P_C[2] <= 0:  # Behind camera
        return 0.0, np.zeros((1, 3), dtype=np.float32)
    
    # Calculate bearing from camera optical axis
    # This gives bearing in camera frame
    camera_bearing = math.atan2(P_C[0], P_C[2])  # atan2(X_cam, Z_cam)
    
    # Convert to bearing from robot heading (if needed)
    # Note: This assumes camera is aligned with robot heading
    z_pred = wrap_to_pi(camera_bearing)
    
    # For H matrix, we can use the analytical approach or numerical
    # Here we use analytical approach in world coordinates
    x_robot, y_robot, theta_robot = x_pred
    x_landmark, y_landmark = centre_W[0], centre_W[1]
    
    Delta_x = x_landmark - x_robot
    Delta_y = y_landmark - y_robot
    r_squared = Delta_x**2 + Delta_y**2
    
    if r_squared < 1e-6:
        return 0.0, np.zeros((1, 3), dtype=np.float32)
    
    # Analytical H matrix (same as above)
    dh_dx = Delta_y / r_squared
    dh_dy = -Delta_x / r_squared  
    dh_dtheta = -1.0
    
    H = np.array([[dh_dx, dh_dy, dh_dtheta]], dtype=np.float32)
    
    return z_pred, H

