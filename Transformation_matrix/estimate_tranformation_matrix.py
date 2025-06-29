import numpy as np
import pandas as pd
import cv2
from scipy.optimize import minimize
from typing import Tuple, List, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class LiDARCameraCalibrator:
    """
    Estimate extrinsic matrix (R, t) between LiDAR and camera using:
    - 3D LiDAR points that lie on chessboard planes
    - Plane equations of chessboards detected in camera frames
    """
    
    def __init__(self):
        self.lidar_points = []  # List of 3D points in LiDAR frame
        self.camera_planes = []  # List of plane equations in camera frame
        self.R = None  # Rotation matrix (3x3)
        self.t = None  # Translation vector (3x1)
        
    def load_data_from_csv(self, csv_path: str) -> None:
        """Load LiDAR points and plane equations from CSV file."""
        df = pd.read_csv(csv_path)
        
        # Group by plane equation to get points per plane
        plane_groups = df.groupby(['a', 'b', 'c', 'd'])
        
        print(f"Loaded {len(df)} points from {len(plane_groups)} planes")
        
        for (a, b, c, d), group in plane_groups:
            # Extract 3D LiDAR points for this plane
            points_3d = group[['x', 'y', 'z']].values
            
            # Store plane equation (normalized)
            plane_normal = np.array([a, b, c])
            plane_normal = plane_normal / np.linalg.norm(plane_normal)
            plane_eq = np.array([*plane_normal, d])
            
            self.lidar_points.append(points_3d)
            self.camera_planes.append(plane_eq)
            
        print(f"Processed {len(self.lidar_points)} plane correspondences")
    
    def transform_points(self, points_3d: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """Transform 3D points from LiDAR to camera frame."""
        return (R @ points_3d.T).T + t.reshape(1, 3)
    
    def point_to_plane_distance(self, points_3d: np.ndarray, plane_eq: np.ndarray) -> np.ndarray:
        """Calculate signed distance from points to plane."""
        a, b, c, d = plane_eq
        return (a * points_3d[:, 0] + b * points_3d[:, 1] + c * points_3d[:, 2] + d)
    
    def objective_function(self, params: np.ndarray) -> float:
        """
        Objective function to minimize: sum of squared distances from 
        transformed LiDAR points to corresponding camera planes.
        
        params: [rx, ry, rz, tx, ty, tz] - rotation vector and translation
        """
        # Extract rotation vector and translation
        rvec = params[:3]
        tvec = params[3:6]
        
        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        t = tvec
        
        total_error = 0.0
        total_points = 0
        
        for lidar_pts, camera_plane in zip(self.lidar_points, self.camera_planes):
            # Transform LiDAR points to camera frame
            transformed_pts = self.transform_points(lidar_pts, R, t)
            
            # Calculate distances to camera plane
            distances = self.point_to_plane_distance(transformed_pts, camera_plane)
            
            # Sum squared distances
            total_error += np.sum(distances**2)
            total_points += len(distances)
        
        # Return RMS error
        return np.sqrt(total_error / total_points)
    
    def estimate_initial_guess(self) -> np.ndarray:
        """
        Estimate initial guess for optimization using plane-to-plane correspondence.
        This is a rough approximation to get started.
        """
        if len(self.lidar_points) < 2:
            return np.array([0, 0, 0, 0, 0, 0])  # Identity transformation
        
        # Use first two planes to estimate rough transformation
        lidar_pts1 = self.lidar_points[0]
        lidar_pts2 = self.lidar_points[1]
        camera_plane1 = self.camera_planes[0]
        camera_plane2 = self.camera_planes[1]
        
        # Fit planes to LiDAR points
        def fit_plane_to_points(points):
            centroid = np.mean(points, axis=0)
            centered = points - centroid
            U, S, Vt = np.linalg.svd(centered)
            normal = Vt[-1]  # Last row of V (normal to plane)
            d = -np.dot(normal, centroid)
            return np.array([*normal, d])
        
        lidar_plane1 = fit_plane_to_points(lidar_pts1)
        lidar_plane2 = fit_plane_to_points(lidar_pts2)
        
        # Rough alignment of plane normals
        lidar_normal1 = lidar_plane1[:3]
        lidar_normal2 = lidar_plane2[:3]
        camera_normal1 = camera_plane1[:3]
        camera_normal2 = camera_plane2[:3]
        
        # Cross product to get rotation axis
        axis1 = np.cross(lidar_normal1, camera_normal1)
        if np.linalg.norm(axis1) > 1e-6:
            axis1 = axis1 / np.linalg.norm(axis1)
            angle1 = np.arccos(np.clip(np.dot(lidar_normal1, camera_normal1), -1, 1))
            rvec_initial = axis1 * angle1
        else:
            rvec_initial = np.array([0, 0, 0])
        
        # Rough translation estimate
        lidar_centroid = np.mean(lidar_pts1, axis=0)
        tvec_initial = -lidar_centroid  # Rough guess
        
        return np.array([*rvec_initial, *tvec_initial])
    
    def calibrate(self, method='L-BFGS-B') -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Estimate extrinsic parameters using nonlinear optimization.
        
        Returns:
            R: 3x3 rotation matrix
            t: 3x1 translation vector
            final_error: RMS reprojection error
        """
        if not self.lidar_points or not self.camera_planes:
            raise ValueError("No data loaded. Call load_data_from_csv() first.")
        
        print("Starting calibration...")
        
        # Initial guess
        x0 = self.estimate_initial_guess()
        
        print(f"Initial guess: rvec={x0[:3]}, tvec={x0[3:]}")
        print(f"Initial error: {self.objective_function(x0):.6f}")
        
        # Optimization
        result = minimize(
            self.objective_function,
            x0,
            method=method,
            options={'disp': True, 'maxiter': 1000}
        )
        
        if not result.success:
            print(f"Warning: Optimization did not converge: {result.message}")
        
        # Extract final parameters
        rvec_final = result.x[:3]
        tvec_final = result.x[3:]
        
        # Convert to rotation matrix
        R_final, _ = cv2.Rodrigues(rvec_final)
        t_final = tvec_final.reshape(3, 1)
        
        final_error = result.fun
        
        self.R = R_final
        self.t = t_final
        
        print(f"\nCalibration completed!")
        print(f"Final RMS error: {final_error:.6f}")
        print(f"Final rotation vector: {rvec_final}")
        print(f"Final translation vector: {tvec_final}")
        
        return R_final, t_final, final_error
    
    def get_transformation_matrix(self) -> np.ndarray:
        """Get 4x4 homogeneous transformation matrix."""
        if self.R is None or self.t is None:
            raise ValueError("Calibration not performed yet.")
        
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t.ravel()
        return T
    
    def validate_calibration(self) -> None:
        """Validate calibration by checking point-to-plane distances."""
        if self.R is None or self.t is None:
            raise ValueError("Calibration not performed yet.")
        
        print("\n=== Calibration Validation ===")
        
        total_points = 0
        errors = []
        
        for i, (lidar_pts, camera_plane) in enumerate(zip(self.lidar_points, self.camera_planes)):
            # Transform points to camera frame
            transformed_pts = self.transform_points(lidar_pts, self.R, self.t)
            
            # Calculate distances to plane
            distances = self.point_to_plane_distance(transformed_pts, camera_plane)
            
            rms_error = np.sqrt(np.mean(distances**2))
            max_error = np.max(np.abs(distances))
            
            print(f"Plane {i+1}: {len(lidar_pts)} points, RMS={rms_error:.6f}, Max={max_error:.6f}")
            
            errors.extend(distances)
            total_points += len(lidar_pts)
        
        errors = np.array(errors)
        overall_rms = np.sqrt(np.mean(errors**2))
        overall_max = np.max(np.abs(errors))
        
        print(f"\nOverall: {total_points} points, RMS={overall_rms:.6f}, Max={overall_max:.6f}")
        
        # Plot error distribution
        plt.figure(figsize=(10, 6))
        plt.subplot(1, 2, 1)
        plt.hist(errors, bins=50, alpha=0.7)
        plt.xlabel('Point-to-plane distance (m)')
        plt.ylabel('Frequency')
        plt.title('Error Distribution')
        plt.grid(True)
        
        plt.subplot(1, 2, 2)
        plt.plot(errors, 'b.', alpha=0.5)
        plt.xlabel('Point index')
        plt.ylabel('Point-to-plane distance (m)')
        plt.title('Error per Point')
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def visualize_3d(self) -> None:
        """Visualize LiDAR points and transformed points in 3D."""
        if self.R is None or self.t is None:
            raise ValueError("Calibration not performed yet.")
        
        fig = plt.figure(figsize=(15, 5))
        
        # Plot 1: Original LiDAR points
        ax1 = fig.add_subplot(131, projection='3d')
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.lidar_points)))
        
        for i, lidar_pts in enumerate(self.lidar_points):
            ax1.scatter(lidar_pts[:, 0], lidar_pts[:, 1], lidar_pts[:, 2], 
                       c=[colors[i]], label=f'Plane {i+1}', alpha=0.6)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('LiDAR Points (Original Frame)')
        ax1.legend()
        
        # Plot 2: Transformed points
        ax2 = fig.add_subplot(132, projection='3d')
        
        for i, lidar_pts in enumerate(self.lidar_points):
            transformed_pts = self.transform_points(lidar_pts, self.R, self.t)
            ax2.scatter(transformed_pts[:, 0], transformed_pts[:, 1], transformed_pts[:, 2],
                       c=[colors[i]], label=f'Plane {i+1}', alpha=0.6)
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('Transformed Points (Camera Frame)')
        ax2.legend()
        
        # Plot 3: Camera planes
        ax3 = fig.add_subplot(133, projection='3d')
        
        # Draw coordinate axes
        origin = [0, 0, 0]
        axes_length = 0.1
        ax3.quiver(*origin, axes_length, 0, 0, color='r', arrow_length_ratio=0.1, label='X')
        ax3.quiver(*origin, 0, axes_length, 0, color='g', arrow_length_ratio=0.1, label='Y')
        ax3.quiver(*origin, 0, 0, axes_length, color='b', arrow_length_ratio=0.1, label='Z')
        
        # Draw plane normals
        for i, plane_eq in enumerate(self.camera_planes):
            normal = plane_eq[:3] * 0.05  # Scale for visualization
            ax3.quiver(0, 0, 0, normal[0], normal[1], normal[2], 
                      color=colors[i], arrow_length_ratio=0.1, 
                      label=f'Normal {i+1}')
        
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_zlabel('Z (m)')
        ax3.set_title('Camera Frame with Plane Normals')
        ax3.legend()
        
        plt.tight_layout()
        plt.show()

# Example usage
def main():
    """Example of how to use the calibrator."""
    # Initialize calibrator
    calibrator = LiDARCameraCalibrator()
    
    # Load data from CSV
    csv_path = "selected_points.csv"  # Your CSV file path
    try:
        calibrator.load_data_from_csv(csv_path)
    except FileNotFoundError:
        print(f"CSV file '{csv_path}' not found. Please check the file path.")
        return
    
    # Perform calibration
    try:
        R, t, error = calibrator.calibrate()
        
        # Print results
        print("\n=== Calibration Results ===")
        print("Rotation Matrix R:")
        print(R)
        print("\nTranslation Vector t:")
        print(t.ravel())
        print(f"\nRMS Error: {error:.6f} meters")
        
        # Get transformation matrix
        T = calibrator.get_transformation_matrix()
        print("\n4x4 Transformation Matrix (LiDAR to Camera):")
        print(T)
        
        # Validate results
        calibrator.validate_calibration()
        
        # Visualize results
        calibrator.visualize_3d()
        
    except Exception as e:
        print(f"Calibration failed: {e}")

if __name__ == "__main__":
    main()