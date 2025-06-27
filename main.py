#!/usr/bin/env python3
# main.py – play a ROS-bag, overlay LiDAR on the camera image and let
#           the user pick point bands.  CSV is written when the window
#           is finally closed.

from pathlib import Path
import yaml, numpy as np, cv2, rclpy, matplotlib.pyplot as plt
from cv_bridge import CvBridge

# ───── our own modules ────────────────────────────────────────────────
from data_synchronization      import load_synchronized_lidar_camera_data
from from_2d_lidar_to_3d_camera import lidar_polar_to_camera_xyz_pairs
from from_3dTo2d_camera_frame   import project_points_from_pairs
from interactive_window   import *            # ← THIS version
from wrote_data_on_csv          import export_selected_points

# ───── file locations ────────────────────────────────────────────────
BAG_DIR         = Path("./TestBag")
INTRINSICS_YAML = Path("./camera_intrinsics.yaml")
EXTRINSICS_YAML = Path("./camera_extrinsics.yaml")
OUT_CSV         = Path("selected_points.csv")

# ──────────────────────────────────────────────────────────────────────
def load_yaml_matrix(path: Path) -> np.ndarray:
    with path.open() as f:
        return np.array(yaml.safe_load(f)).reshape(4, 4)


def main() -> None:
    # 1.  Camera intrinsics
    with INTRINSICS_YAML.open() as f:
        intr = yaml.safe_load(f)
    K = np.array(intr["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    D = np.array(intr["distortion_coefficients"]["data"], dtype=float)

    # 2.  LiDAR→camera extrinsics
    with EXTRINSICS_YAML.open() as f:
        extr = yaml.safe_load(f)
    T_L2C = np.array(extr["extrinsic_matrix"], dtype=float).reshape(4, 4)

    # 3.  Initialise ROS – *only once* per process
    if not rclpy.ok():
        rclpy.init()

    bridge = CvBridge()
    win    = LiDARCameraWindow(K, D)          # create the UI once
    plt.ion()
    plt.show(block=False)                     # map the window immediately

    last_pairs = None                         # keep latest point list for CSV

    # ── loop over synchronized packets ──────────────────────────────
    for scan_msg, image_msg, stamp in load_synchronized_lidar_camera_data(BAG_DIR):

        # 3-D points in **camera** frame
        pairs = lidar_polar_to_camera_xyz_pairs(
            ranges=np.asarray(scan_msg.ranges),
            angle_min=scan_msg.angle_min,
            angle_increment=scan_msg.angle_increment,
            range_min=scan_msg.range_min,
            range_max=scan_msg.range_max,
            T_lidar_to_camera=T_L2C,
        )
        if not pairs:
            continue
        print(pairs)
        last_pairs = pairs                           # remember for CSV

        scan_idx, uv_px = project_points_from_pairs(pairs, K)  # already pixels
        pts_cam = np.array([xyz for _, xyz in pairs])

        # ROS → RGB numpy
        img_bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # feed one frame
        if not win.push_frame(
            img_rgb   = img_rgb,
            scan_idx  = np.asarray(scan_idx, dtype=int),
            uv        = uv_px.astype(float),
            
            timestamp = stamp,
        ):
            break                                      # user pressed ‘q’

        plt.pause(0.001)                               # let GUI redraw
        if win.quit_requested:                                   # safety check
            break

    # ── block until window is closed, then dump CSV ─────────────────
    selections = win.result()                          # modal – waits for close
    if selections and last_pairs:
        export_selected_points(selections, last_pairs, OUT_CSV)

    if rclpy.ok():
        rclpy.shutdown()
    print("✓ finished")

# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
