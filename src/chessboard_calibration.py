import cv2
import numpy as np
from typing import Optional, Tuple
import csv 

class ChessboardPlaneCalibrator:
    def __init__(self, chessboard_size: Tuple[int, int] = (7, 10), square_size: float = 0.025):
        """
        Initialize chessboard calibrator.
        
        Args:
            chessboard_size: (width, height) number of internal corners
            square_size: Physical size of each square in meters
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # Generate 3D chessboard points in chessboard coordinate system
        self.object_points = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.object_points *= square_size

    def detect_chessboard(self, image: np.ndarray) -> Optional[np.ndarray]:
        """
        Locate the inner-corner grid of the calibration chessboard.

        Returns
        -------
        np.ndarray  (N×2)   sub-pixel corner coordinates (u, v) **in image
        None                 if the board is not found
        """
        # 1.  Gray image
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image

        # 2.  Try the modern SB detector first  (OpenCV ≥ 4.5)
        ret, corners = cv2.findChessboardCornersSB(
            gray,
            self.chessboard_size,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        )

        # 3.  Fallback to the classic detector if SB fails
        if not ret:
            ret, corners = cv2.findChessboardCorners(
                gray,
                self.chessboard_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
            )

        if not ret:
            return None  # board absent → caller handles gracefully

        # 4.  Corner sub-pixel refinement (classic OpenCV recipe)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,   # max iterations
                    0.001 # epsilon
                   )
        corners = cv2.cornerSubPix(
            gray, corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria
        )

        # 5.  Return as (N,2) float32 array
        return corners.reshape(-1, 2)

    def estimate_chessboard_pose(self, image_corners: np.ndarray, camera_matrix: np.ndarray, 
                                dist_coeffs: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Estimate chessboard pose using PnP.
        
        Returns:
            (rotation_vector, translation_vector) or None if failed
        """
        success, rvec, tvec = cv2.solvePnP(
            self.object_points, 
            image_corners, 
            camera_matrix, 
            dist_coeffs
        )
        
        if success:
            return rvec, tvec
        return None

    def get_chessboard_plane_equation(self, rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Get plane equation from chessboard pose.
        
        Returns:
            (normal_vector, d) where plane equation is n·x + d = 0
        """
        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        
        # Chessboard normal in camera frame (Z-axis of chessboard)
        normal_chessboard = np.array([0, 0, 1])
        normal_camera = R @ normal_chessboard
        
        # Normalize
        normal_camera = normal_camera / np.linalg.norm(normal_camera)
        
        # Plane equation: n·(x - p) = 0, where p is a point on the plane
        # Expanding: n·x - n·p = 0, so d = -n·p
        d = -np.dot(normal_camera, tvec.flatten())
        
        return normal_camera, d

# Modified save_selection_to_file method for your InteractiveLiDARVisualizer class


def save_selection_to_file_chessboard(self, indices, distances, uvs, frame_number):
    """Filter LiDAR points by distance outliers (per frame) and save valid 2D-3D pairs."""

    angle_min = self.current_scan.angle_min
    angle_inc = self.current_scan.angle_increment

    all_points_3d = []
    for idx in indices:
        dist = self.current_ranges[idx]
        angle = angle_min + idx * angle_inc
        x = dist * np.cos(angle)
        y = dist * np.sin(angle)
        z = 0.0
        all_points_3d.append([x, y, z])
    all_points_3d = np.array(all_points_3d)
    all_dists = np.array([self.current_ranges[idx] for idx in indices])

    # === Compute mean and std deviation of distances
    mean_dist = np.mean(all_dists)
    std_dist = np.std(all_dists)
    threshold = 2.0  # keep points within 2 std devs

    mask = np.abs(all_dists - mean_dist) < threshold * std_dist

    # Filter all arrays
    filtered_points_3d = all_points_3d[mask]
    filtered_uvs = [uv for uv, keep in zip(uvs, mask) if keep]
    filtered_indices = [i for i, keep in zip(indices, mask) if keep]
    filtered_distances = [d for d, keep in zip(distances, mask) if keep]

    print(f"[INFO] Frame {frame_number}: {np.sum(mask)} / {len(mask)} LiDAR points kept")

    if len(filtered_points_3d) == 0:
        print(f"[WARN] All points in frame {frame_number} filtered out.")
        return

    # === Detect chessboard and estimate plane
    if hasattr(self, 'chessboard_calibrator') and self.current_frame is not None:
        corners = self.chessboard_calibrator.detect_chessboard(self.current_frame)
        if corners is not None:
            K = np.array(self.cam_info.k).reshape(3, 3)
            dist_coeffs = np.array(self.cam_info.d)
            pose = self.chessboard_calibrator.estimate_chessboard_pose(corners, K, dist_coeffs)
            if pose is not None:
                rvec, tvec = pose
                normal, d = self.chessboard_calibrator.get_chessboard_plane_equation(rvec, tvec)
                a, b, c = normal
            else:
                a = b = c = d = 0.0
        else:
            a = b = c = d = 0.0
    else:
        a = b = c = d = 0.0

    # === Save to CSV
    with open(self.output_csv, 'a', newline='') as f:
        writer = csv.writer(f)
        for (idx, dist, (u, v), (x, y, z)) in zip(filtered_indices, filtered_distances, filtered_uvs, filtered_points_3d):
            writer.writerow([u, v, x, y, z, d, a, b, c])

# Integration into your InteractiveLiDARVisualizer class
def integrate_chessboard_calibrator(visualizer_class):
    """
    Add chessboard calibration capability to your visualizer.
    Call this modification in your __init__ method.
    """
    
    # Add to __init__ method:
    def enhanced_init(self, cam_info, chessboard_size=(9, 6), square_size=0.025, output_csv="lidar_selection_log.csv"):
        # ... existing init code ...
        
        # Add chessboard calibrator
        self.chessboard_calibrator = ChessboardPlaneCalibrator(chessboard_size, square_size)
        
        # Replace the save method
        self.save_selection_to_file = save_selection_to_file_chessboard.__get__(self, type(self))
    
    return enhanced_init

# Example usage in your main function:
def main_with_chessboard():
    # ... existing code ...
    
    # When creating visualizer, specify chessboard parameters
    visualizer = InteractiveLiDARVisualizer(
        cam_info, 
        chessboard_size=(7, 10),  # Adjust to your chessboard
        square_size=0.025        # Adjust to your square size in meters
    )
    
    # ... rest of your main function ...