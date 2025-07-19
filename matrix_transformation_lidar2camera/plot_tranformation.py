import numpy as np
import matplotlib.pyplot as plt

# === Estimated Rotation and Translation ===
R_est = np.array([
    [ 0.96903375, -0.05349284, -0.24106455],
    [ 0.23855629, -0.04924747,  0.96987916],
    [-0.06375341, -0.99735310, -0.03496141]
])
t_est = np.array([ 0.05574216, -0.01709673, -0.01984395])

# === ROS2 Extrinsic Matrix (YAML-style) ===
T_ros2 = np.array([
    [-1.0, -3.14018491736755e-16,  2.465190328815662e-32,  1.3877787807814457e-17],
    [ 2.465190328815662e-32, -2.220446049250313e-16, -1.0,  0.0463935],
    [ 3.14018491736755e-16, -1.0,  2.220446049250313e-16,  0.04099558],
    [0.0, 0.0, 0.0, 1.0]
])
R_ros2 = T_ros2[:3, :3]
t_ros2 = T_ros2[:3, 3]

# === Fix ROS2 frame: Apply 180° flip over Z-axis ===
R_flip = np.array([
    [-1, 0,  0],
    [ 0, -1, 0],
    [ 0,  0, 1]
])
R_ros2_fixed = R_flip @ R_ros2
t_ros2_fixed = R_flip @ t_ros2

# === Compute angle error ===
R_diff = R_ros2_fixed.T @ R_est
cos_theta = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
rot_error_rad = np.arccos(cos_theta)
rot_error_deg = np.degrees(rot_error_rad)
trans_error = np.linalg.norm(t_est - t_ros2_fixed)

print(f"Rotation difference after fix: {rot_error_deg:.2f} degrees")
print(f"Translation difference after fix: {trans_error:.4f} meters")

# === Helper: draw a frame ===
def draw_frame(ax, origin, R, label_prefix, colors, alpha=1.0):
    x, y, z = R[:, 0], R[:, 1], R[:, 2]
    ax.quiver(*origin, *x, length=0.1, color=colors[0], alpha=alpha, label=f'{label_prefix} X')
    ax.quiver(*origin, *y, length=0.1, color=colors[1], alpha=alpha, label=f'{label_prefix} Y')
    ax.quiver(*origin, *z, length=0.1, color=colors[2], alpha=alpha, label=f'{label_prefix} Z')

# === 3D Plot ===
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Reference vs ROS2 vs Estimated Frames")

# Color sets for each frame (X, Y, Z)
ref_colors = ('gray', 'gray', 'gray')            # Reference frame
ros2_colors = ('orange', 'purple', 'cyan')       # ROS2 corrected
est_colors = ('darkred', 'darkgreen', 'navy')    # Estimated frame

# Draw all frames
draw_frame(ax, np.zeros(3), np.eye(3), 'Ref', colors=ref_colors, alpha=0.4)
draw_frame(ax, t_ros2_fixed, R_ros2_fixed, 'ROS2', colors=ros2_colors, alpha=0.9)
draw_frame(ax, t_est, R_est, 'Est', colors=est_colors, alpha=1.0)

# Axis settings
ax.set_xlim([-0.1, 0.1])
ax.set_ylim([-0.1, 0.1])
ax.set_zlim([-0.1, 0.1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Optional: remove duplicate labels
handles, labels = ax.get_legend_handles_labels()
unique = dict(zip(labels, handles))
ax.legend(unique.values(), unique.keys(), loc='upper left')

plt.show()

