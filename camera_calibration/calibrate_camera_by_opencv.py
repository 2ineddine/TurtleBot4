# import cv2
# import numpy as np
# import os
# import glob
# import yaml

# # ─── Checkerboard Setup ─────────────────────────
# CHECKERBOARD = (7, 10)
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# objpoints = []  # 3D points in real world
# imgpoints = []  # 2D points in image plane

# # Generate object points for the checkerboard
# objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# # ─── Load Calibration Images ────────────────────
# images = glob.glob('./*.png')
# if not images:
#     raise RuntimeError("No calibration images (*.png) found in current directory.")

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     ret, corners = cv2.findChessboardCorners(
#         gray, CHECKERBOARD,
#         cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
#     )

#     if ret:
#         objpoints.append(objp)
#         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#         imgpoints.append(corners2)

#         img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
#         cv2.imshow('Chessboard Detection', img)
#         cv2.waitKey(200)

# cv2.destroyAllWindows()

# # ─── Calibrate Camera ───────────────────────────
# ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, gray.shape[::-1], None, None
# )

# print("[INFO] Calibration successful")
# print("Camera Matrix (K):\n", K)
# print("Distortion Coefficients:\n", dist)

# # ─── Save to camera_intrinsics.yaml ─────────────
# output_data = {
#     "camera_matrix": {
#         "rows": 3,
#         "cols": 3,
#         "data": K.flatten().tolist()
#     },
#     "distortion_model": "plumb_bob",
#     "distortion_coefficients": {
#         "rows": 1,
#         "cols": len(dist.flatten()),
#         "data": dist.flatten().tolist()
#     }
# }

# with open("camera_intrinsics.yaml", "w") as f:
#     yaml.dump(output_data, f)

# print("[INFO] Camera intrinsics saved to: camera_intrinsics.yaml")
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2
# import cv2
# import numpy as np
# import os
# import glob
# import yaml
# import matplotlib.pyplot as plt

# # ─── Checkerboard Setup ─────────────────────────
# CHECKERBOARD = (7, 10)
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# objpoints = []  # 3D points in real world
# imgpoints = []  # 2D points in image plane
# image_names = []  # Store image names for error analysis

# # Generate object points for the checkerboard
# objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# # ─── Load Calibration Images ────────────────────
# images = glob.glob('./*.png')
# if not images:
#     raise RuntimeError("No calibration images (*.png) found in current directory.")

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     ret, corners = cv2.findChessboardCorners(
#         gray, CHECKERBOARD,
#         cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
#     )
#     if ret:
#         objpoints.append(objp)
#         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#         imgpoints.append(corners2)
#         image_names.append(os.path.basename(fname))  # Store image name
        
#         img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
#         cv2.imshow('Chessboard Detection', img)
#         cv2.waitKey(200)

# cv2.destroyAllWindows()

# # ─── Calibrate Camera ───────────────────────────
# ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, gray.shape[::-1], None, None
# )

# print("[INFO] Calibration successful")
# print(f"[INFO] Overall RMS reprojection error: {ret:.4f} pixels")
# print("Camera Matrix (K):\n", K)
# print("Distortion Coefficients:\n", dist)

# # ─── Calculate Reprojection Errors ─────────────
# def calculate_reprojection_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
#     """Calculate reprojection errors for each image and point"""
#     total_error = 0
#     errors_per_image = []
#     max_errors_per_image = []
    
#     for i in range(len(objpoints)):
#         # Project 3D points to 2D
#         projected_points, _ = cv2.projectPoints(
#             objpoints[i], rvecs[i], tvecs[i], K, dist
#         )
        
#         # Calculate error for each point
#         error = cv2.norm(imgpoints[i], projected_points, cv2.NORM_L2) / len(projected_points)
#         errors_per_image.append(error)
        
#         # Calculate per-point errors for this image
#         point_errors = []
#         for j in range(len(projected_points)):
#             point_error = cv2.norm(imgpoints[i][j], projected_points[j], cv2.NORM_L2)
#             point_errors.append(point_error)
        
#         max_error = np.max(point_errors)
#         max_errors_per_image.append(max_error)
#         total_error += error
    
#     mean_error = total_error / len(objpoints)
#     return errors_per_image, max_errors_per_image, mean_error

# # Calculate errors
# errors_per_image, max_errors_per_image, mean_error = calculate_reprojection_errors(
#     objpoints, imgpoints, rvecs, tvecs, K, dist
# )

# # ─── Display Error Statistics ─────────────────
# print("\n" + "="*60)
# print("REPROJECTION ERROR ANALYSIS")
# print("="*60)
# print(f"Mean reprojection error: {mean_error:.4f} pixels")
# print(f"Standard deviation: {np.std(errors_per_image):.4f} pixels")
# print(f"Minimum error: {np.min(errors_per_image):.4f} pixels")
# print(f"Maximum error: {np.max(errors_per_image):.4f} pixels")
# print(f"Maximum single point error: {np.max(max_errors_per_image):.4f} pixels")

# print("\nPer-image error analysis:")
# print("-" * 50)
# for i, (name, error, max_err) in enumerate(zip(image_names, errors_per_image, max_errors_per_image)):
#     status = "✓ Good" if error < 0.5 else "⚠ Warning" if error < 1.0 else "✗ Poor"
#     print(f"{name:20s} | Mean: {error:.4f} | Max: {max_err:.4f} | {status}")

# # ─── Quality Assessment ─────────────────────────
# def assess_calibration_quality(rms_error, errors_per_image):
#     """Assess overall calibration quality"""
#     print("\n" + "="*60)
#     print("CALIBRATION QUALITY ASSESSMENT")
#     print("="*60)
    
#     # Overall quality
#     if rms_error < 0.3:
#         quality = "Excellent"
#     elif rms_error < 0.5:
#         quality = "Good"
#     elif rms_error < 1.0:
#         quality = "Acceptable"
#     else:
#         quality = "Poor"
    
#     print(f"Overall Quality: {quality} (RMS = {rms_error:.4f} pixels)")
    
#     # Image consistency
#     std_dev = np.std(errors_per_image)
#     if std_dev < 0.1:
#         consistency = "Very consistent"
#     elif std_dev < 0.2:
#         consistency = "Consistent"
#     elif std_dev < 0.3:
#         consistency = "Moderately consistent"
#     else:
#         consistency = "Inconsistent"
    
#     print(f"Consistency: {consistency} (σ = {std_dev:.4f})")
    
#     # Recommendations
#     print("\nRecommendations:")
#     if rms_error > 0.5:
#         print("- Consider adding more calibration images")
#         print("- Ensure better coverage of the image area")
#         print("- Check for motion blur or poor focus")
    
#     if std_dev > 0.2:
#         print("- Some images have significantly higher errors")
#         print("- Consider removing worst-performing images")
    
#     if rms_error < 0.3 and std_dev < 0.1:
#         print("- Calibration quality is excellent!")
#         print("- Ready for precise 3D measurements")

# assess_calibration_quality(ret, errors_per_image)

# # ─── Visualization ─────────────────────────────
# def plot_error_analysis():
#     """Create visualization plots for error analysis"""
#     fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
#     # Plot 1: Error per image
#     ax1.bar(range(len(errors_per_image)), errors_per_image, alpha=0.7, color='steelblue')
#     ax1.axhline(y=0.5, color='orange', linestyle='--', label='Good threshold (0.5 px)')
#     ax1.axhline(y=1.0, color='red', linestyle='--', label='Acceptable threshold (1.0 px)')
#     ax1.set_xlabel('Image Index')
#     ax1.set_ylabel('RMS Error (pixels)')
#     ax1.set_title('Reprojection Error per Image')
#     ax1.legend()
#     ax1.grid(True, alpha=0.3)
    
#     # Plot 2: Error histogram
#     ax2.hist(errors_per_image, bins=10, alpha=0.7, color='lightcoral', edgecolor='black')
#     ax2.axvline(x=np.mean(errors_per_image), color='red', linestyle='-', linewidth=2, label=f'Mean: {np.mean(errors_per_image):.3f}')
#     ax2.axvline(x=np.median(errors_per_image), color='green', linestyle='-', linewidth=2, label=f'Median: {np.median(errors_per_image):.3f}')
#     ax2.set_xlabel('RMS Error (pixels)')
#     ax2.set_ylabel('Frequency')
#     ax2.set_title('Error Distribution')
#     ax2.legend()
#     ax2.grid(True, alpha=0.3)
    
#     # Plot 3: Max error per image
#     ax3.bar(range(len(max_errors_per_image)), max_errors_per_image, alpha=0.7, color='mediumpurple')
#     ax3.axhline(y=1.0, color='orange', linestyle='--', label='Warning threshold (1.0 px)')
#     ax3.axhline(y=2.0, color='red', linestyle='--', label='Poor threshold (2.0 px)')
#     ax3.set_xlabel('Image Index')
#     ax3.set_ylabel('Max Point Error (pixels)')
#     ax3.set_title('Maximum Point Error per Image')
#     ax3.legend()
#     ax3.grid(True, alpha=0.3)
    
#     # Plot 4: Error statistics summary
#     stats = [np.mean(errors_per_image), np.median(errors_per_image), 
#              np.std(errors_per_image), np.max(errors_per_image)]
#     labels = ['Mean', 'Median', 'Std Dev', 'Maximum']
#     colors = ['steelblue', 'green', 'orange', 'red']
    
#     bars = ax4.bar(labels, stats, color=colors, alpha=0.7)
#     ax4.set_ylabel('Error (pixels)')
#     ax4.set_title('Error Statistics Summary')
#     ax4.grid(True, alpha=0.3)
    
#     # Add value labels on bars
#     for bar, stat in zip(bars, stats):
#         height = bar.get_height()
#         ax4.text(bar.get_x() + bar.get_width()/2., height + 0.01,
#                 f'{stat:.3f}', ha='center', va='bottom')
    
#     plt.tight_layout()
#     plt.savefig('calibration_error_analysis.png', dpi=300, bbox_inches='tight')
#     plt.show()

# # Generate plots
# plot_error_analysis()

# # ─── Save Detailed Results ─────────────────────
# results_data = {
#     "calibration_results": {
#         "overall_rms_error": float(ret),
#         "mean_error": float(mean_error),
#         "std_deviation": float(np.std(errors_per_image)),
#         "min_error": float(np.min(errors_per_image)),
#         "max_error": float(np.max(errors_per_image)),
#         "max_single_point_error": float(np.max(max_errors_per_image)),
#         "num_images": len(errors_per_image),
#         "per_image_errors": [float(e) for e in errors_per_image],
#         "image_names": image_names
#     }
# }

# # ─── Save to camera_intrinsics.yaml ─────────────
# output_data = {
#     "camera_matrix": {
#         "rows": 3,
#         "cols": 3,
#         "data": K.flatten().tolist()
#     },
#     "distortion_model": "plumb_bob",
#     "distortion_coefficients": {
#         "rows": 1,
#         "cols": len(dist.flatten()),
#         "data": dist.flatten().tolist()
#     },
#     "calibration_quality": results_data["calibration_results"]
# }

# with open("camera_intrinsics.yaml", "w") as f:
#     yaml.dump(output_data, f)

# print(f"\n[INFO] Camera intrinsics saved to: camera_intrinsics.yaml")
# print(f"[INFO] Error analysis plot saved to: calibration_error_analysis.png")
# print(f"[INFO] Calibration completed with {len(image_names)} images")
#@@@
# Version simplifiée avec filtrage agressif des outliers
import cv2
import numpy as np
import os
import glob
import yaml
import matplotlib.pyplot as plt

CHECKERBOARD = (7, 10)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objpoints = []
imgpoints = []
image_names = []

# Generate object points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

def remove_outlier_points_simple(objpoints_list, imgpoints_list, max_error_threshold=1.0):
    """Remove outlier points based on initial calibration"""
    if len(objpoints_list) == 0:
        return objpoints_list, imgpoints_list
    
    # Perform initial calibration
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints_list, imgpoints_list, (640, 480), None, None
    )
    
    filtered_objpoints = []
    filtered_imgpoints = []
    
    print(f"[INFO] Initial RMS error: {ret:.4f} pixels")
    print(f"[INFO] Filtering points with error > {max_error_threshold} pixels")
    
    for i in range(len(objpoints_list)):
        # Project points back to image
        projected_points, _ = cv2.projectPoints(
            objpoints_list[i], rvecs[i], tvecs[i], K, dist
        )
        
        # Calculate per-point errors
        original_points = imgpoints_list[i].reshape(-1, 2)
        projected_flat = projected_points.reshape(-1, 2)
        
        valid_indices = []
        for j in range(len(original_points)):
            error = cv2.norm(original_points[j], projected_flat[j])
            if error <= max_error_threshold:
                valid_indices.append(j)
        
        # Keep only valid points
        if len(valid_indices) >= 50:  # Minimum 50 points per image
            filtered_img_pts = original_points[valid_indices].reshape(-1, 1, 2)
            filtered_obj_pts = objpoints_list[i].reshape(-1, 3)[valid_indices].reshape(1, -1, 3)
            
            filtered_imgpoints.append(filtered_img_pts)
            filtered_objpoints.append(filtered_obj_pts)
    
    return filtered_objpoints, filtered_imgpoints

# Load images and detect corners
images = glob.glob('./*.png')
print(f"[INFO] Processing {len(images)} images...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)
    
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        image_names.append(os.path.basename(fname))

# Apply filtering to get BEST version
print("\n[INFO] Applying intelligent filtering...")
filtered_objpoints, filtered_imgpoints = remove_outlier_points_simple(
    objpoints, imgpoints, max_error_threshold=1.7
)

# Calibrate with filtered data (BEST version)
ret_best, K_best, dist_best, rvecs_best, tvecs_best = cv2.calibrateCamera(
    filtered_objpoints, filtered_imgpoints, (640, 480), None, None
)

# Calculate all point errors for the BEST version
all_point_errors = []
for i in range(len(filtered_objpoints)):
    projected_points, _ = cv2.projectPoints(
        filtered_objpoints[i], rvecs_best[i], tvecs_best[i], K_best, dist_best
    )
    
    original_points = filtered_imgpoints[i].reshape(-1, 2)
    projected_flat = projected_points.reshape(-1, 2)
    
    for j in range(len(original_points)):
        point_error = cv2.norm(original_points[j], projected_flat[j])
        all_point_errors.append(point_error)

# ═══════════════════════════════════════════════════════════════
# MEILLEUR GRAPHIQUE DE LA MEILLEURE VERSION
# ═══════════════════════════════════════════════════════════════

plt.figure(figsize=(12, 8))

# Create the BEST quality analysis graph
plt.hist(all_point_errors, bins=50, alpha=0.7, color='lightgreen', 
         edgecolor='darkgreen', linewidth=1.5, density=True)

# Add statistical lines
mean_error = np.mean(all_point_errors)
median_error = np.median(all_point_errors)
percentile_95 = np.percentile(all_point_errors, 95)
percentile_99 = np.percentile(all_point_errors, 99)
max_error = np.max(all_point_errors)

plt.axvline(x=mean_error, color='red', linestyle='-', linewidth=3, 
           label=f'Mean: {mean_error:.3f} px')
plt.axvline(x=median_error, color='blue', linestyle='-', linewidth=3, 
           label=f'Median: {median_error:.3f} px')
plt.axvline(x=percentile_95, color='orange', linestyle='--', linewidth=2, 
           label=f'95th percentile: {percentile_95:.3f} px')
plt.axvline(x=percentile_99, color='purple', linestyle='--', linewidth=2, 
           label=f'99th percentile: {percentile_99:.3f} px')

# Quality thresholds
plt.axvline(x=0.5, color='green', linestyle=':', linewidth=2, alpha=0.7,
           label='Excellent threshold (0.5 px)')
plt.axvline(x=1.0, color='yellow', linestyle=':', linewidth=2, alpha=0.7,
           label='Good threshold (1.0 px)')

plt.xlabel('Point Reprojection Error (pixels)', fontsize=14, fontweight='bold')
plt.ylabel('Density', fontsize=14, fontweight='bold')
plt.title(f'Distribution of Point Errors - BEST CALIBRATION QUALITY\n'
          f'RMS Error: {ret_best:.4f} pixels | Total Points: {len(all_point_errors)}', 
          fontsize=16, fontweight='bold')

plt.legend(fontsize=12, loc='upper right')
plt.grid(True, alpha=0.3)

# Add quality assessment text box
quality_text = f"""
CALIBRATION QUALITY ASSESSMENT:
• RMS Error: {ret_best:.4f} pixels
• Mean Error: {mean_error:.4f} pixels  
• 95% of points < {percentile_95:.3f} pixels
• Max Error: {max_error:.3f} pixels
• Images Used: {len(filtered_objpoints)}
• Total Points: {len(all_point_errors)}

VERDICT: {'EXCELLENT' if ret_best < 0.5 else 'GOOD' if ret_best < 1.0 else 'ACCEPTABLE'}
"""



plt.tight_layout()
plt.savefig('BEST_calibration_quality_analysis.png', dpi=300, bbox_inches='tight')
plt.show()

# Print summary
print(f"\n" + "="*60)
print("BEST CALIBRATION RESULTS")
print("="*60)
print(f"RMS Error: {ret_best:.4f} pixels")
print(f"Mean Error: {mean_error:.4f} pixels")
print(f"95th Percentile: {percentile_95:.4f} pixels")
print(f"Max Point Error: {max_error:.4f} pixels")
print(f"Images Used: {len(filtered_objpoints)}")
print(f"Total Points Analyzed: {len(all_point_errors)}")

if ret_best < 0.3:
    quality = "EXCELLENT"
elif ret_best < 0.5:
    quality = "VERY GOOD"
elif ret_best < 1.0:
    quality = "GOOD"
else:
    quality = "NEEDS IMPROVEMENT"

print(f"Overall Quality: {quality}")
print(f"Ready for precision applications: {'YES' if ret_best < 0.5 else 'MAYBE !'}")

# Save the best calibration
output_data = {
    "camera_matrix": {
        "rows": 3, "cols": 3,
        "data": K_best.flatten().tolist()
    },
    "distortion_model": "plumb_bob", 
    "distortion_coefficients": {
        "rows": 1, "cols": len(dist_best.flatten()),
        "data": dist_best.flatten().tolist()
    },
    "quality_metrics": {
        "rms_error": float(ret_best),
        "mean_error": float(mean_error),
        "percentile_95": float(percentile_95),
        "max_error": float(max_error)

    }
}

with open("camera_intrinsics.yaml", "w") as f:
    yaml.dump(output_data, f)

print(f"\n[INFO] Best calibration saved to: camera_intrinsics_BEST.yaml")
print(f"[INFO] Quality analysis graph saved to: BEST_calibration_quality_analysis.png")
