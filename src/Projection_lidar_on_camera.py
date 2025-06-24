# """
# Interactive LiDAR Point Cloud Visualizer with Zoom and Selection - Camera View Only
# Modified version with single camera window and controlled zoom/selection functionality.

# Controls:
# - 'p': Pause/unpause the video stream
# - 'z': Enable/disable zoom mode (only when paused)
# - Left click: Select LiDAR point (only when paused and zoom enabled)
# - 'q': Quit application
# - Space: Step to next frame when paused
# - When zoom enabled: Use matplotlib's zoom/pan tools
# """
# from __future__ import annotations

# import argparse
# import sys
# from pathlib import Path
# from typing import Dict, List, Tuple, Optional, Any

# import cv2
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# import numpy as np
# import rclpy
# from rclpy.serialization import deserialize_message
# from builtin_interfaces.msg import Time as TimeMsg
# from cv_bridge import CvBridge
# from geometry_msgs.msg import TransformStamped
# from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, TopicMetadata
# from sensor_msgs.msg import CameraInfo, LaserScan, Image
# from tf2_msgs.msg import TFMessage
# from tf2_ros import Buffer, LookupException, ConnectivityException, ExtrapolationException

# # ────────────────────────────────────────── Interactive Visualizer Class ────────────────────────────

# class InteractiveLiDARVisualizer:
#     def __init__(self, cam_info: CameraInfo):
#         self.cam_info = cam_info
#         self.paused = False
#         self.quit_requested = False
#         self.zoom_enabled = False
#         self.current_frame = None
#         self.current_lidar_points = None
#         self.current_uv_points = None
#         self.selected_point = None
#         self.selected_point_idx = None
#         self.selected_point_3d = None  # Store 3D coordinates
        
#         # Setup matplotlib figure with single subplot
#         self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 8))
#         self.fig.suptitle('Interactive LiDAR Overlay Visualizer - Camera View')
        
#         # Camera subplot
#         self.ax.set_title('Camera Feed with LiDAR Overlay')
#         self.ax.set_aspect('equal')
        
#         # Connect event handlers
#         self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
#         self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        
#         # Text display for selected point info and controls
#         self.info_text = self.fig.text(0.02, 0.98, '', transform=self.fig.transFigure, 
#                                       verticalalignment='top', fontsize=10,
#                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
#         self.update_info_text()
        
#     def update_info_text(self):
#         """Update the information text display."""
#         info = "Controls:\n"
#         info += "'p': Pause/Unpause | 'q': Quit | Space: Next frame (when paused)\n"
#         info += "'z': Enable/Disable zoom mode (when paused) | Left click: Select point (when paused & zoom enabled)\n"
        
#         status_parts = []
#         if self.paused:
#             status_parts.append("PAUSED")
#         else:
#             status_parts.append("PLAYING")
            
#         if self.zoom_enabled:
#             status_parts.append("ZOOM ENABLED")
            
#         info += f"Status: {' | '.join(status_parts)}\n"
        
#         if self.selected_point_3d is not None:
#             x, y, z = self.selected_point_3d
#             info += f"Selected Point 3D: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m\n"
            
#         if self.selected_point is not None:
#             u, v = self.selected_point
#             info += f"Selected Point 2D: U={u:.0f}px, V={v:.0f}px"
        
#         self.info_text.set_text(info)
    
#     def on_key_press(self, event):
#         """Handle keyboard events."""
#         if event.key == 'p':
#             self.paused = not self.paused
#             if not self.paused:
#                 # When resuming, disable zoom mode
#                 self.zoom_enabled = False
#                 # Reset matplotlib toolbar to default state
#                 self.fig.canvas.toolbar.mode = ''
#             print(f"{'Paused' if self.paused else 'Resumed'}")
#             self.update_info_text()
            
#         elif event.key == 'z':
#             if self.paused:
#                 self.zoom_enabled = not self.zoom_enabled
#                 if self.zoom_enabled:
#                     print("Zoom mode enabled - You can now use matplotlib's zoom/pan tools and click to select points")
#                 else:
#                     print("Zoom mode disabled")
#                     # Reset matplotlib toolbar
#                     self.fig.canvas.toolbar.mode = ''
#                 self.update_info_text()
#             else:
#                 print("Zoom mode can only be toggled when video is paused")
                
#         elif event.key == 'q':
#             self.quit_requested = True
#             print("Quit requested")
            
#         elif event.key == ' ' and self.paused:
#             # Space bar advances one frame when paused
#             print("Advance one frame")
    
#     def on_mouse_click(self, event):
#         """Handle mouse click events for point selection."""
#         # Only allow selection when paused and zoom is enabled
#         if not self.paused or not self.zoom_enabled:
#             return
            
#         if event.inaxes != self.ax or self.current_uv_points is None:
#             return
            
#         if event.button == 1:  # Left click - select point
#             self.select_nearest_point(event.xdata, event.ydata)
    
#     def select_nearest_point(self, click_x, click_y):
#         """Select the nearest LiDAR point to the click location in image coordinates."""
#         if self.current_uv_points is None or self.current_lidar_points is None:
#             return
        
#         # Filter points that are visible in the image
#         img_height, img_width = self.current_frame.shape[:2]
#         in_img = ((self.current_uv_points[:, 0] >= 0) & (self.current_uv_points[:, 0] < img_width) & 
#                  (self.current_uv_points[:, 1] >= 0) & (self.current_uv_points[:, 1] < img_height))
        
#         if not np.any(in_img):
#             return
            
#         visible_uv = self.current_uv_points[in_img]
#         visible_3d = self.current_lidar_points[in_img]
        
#         # Calculate distances to all visible points in image coordinates
#         distances = np.sqrt((visible_uv[:, 0] - click_x)**2 + (visible_uv[:, 1] - click_y)**2)
        
#         # Find nearest point
#         nearest_idx = np.argmin(distances)
        
#         # Store both 2D and 3D coordinates
#         self.selected_point = (visible_uv[nearest_idx, 0], visible_uv[nearest_idx, 1])
#         self.selected_point_3d = (visible_3d[nearest_idx, 0], visible_3d[nearest_idx, 1], visible_3d[nearest_idx, 2])
        
#         # Find the original index in the full arrays
#         original_indices = np.where(in_img)[0]
#         self.selected_point_idx = original_indices[nearest_idx]
        
#         print(f"Selected point: 2D=({self.selected_point[0]:.0f}, {self.selected_point[1]:.0f})px, "
#               f"3D=({self.selected_point_3d[0]:.3f}, {self.selected_point_3d[1]:.3f}, {self.selected_point_3d[2]:.3f})m")
        
#         self.update_info_text()
        
#         # Update display to show selected point
#         self.update_display_with_selection()
    
#     def update_display(self, img_rgb, lidar_points_cam, uv_points):
#         """Update camera display."""
#         if self.quit_requested:
#             return False
        
#         self.current_frame = img_rgb
#         self.current_lidar_points = lidar_points_cam
#         self.current_uv_points = uv_points
        
#         # Update display
#         self.update_camera_display()
        
#         plt.tight_layout()
#         self.fig.canvas.draw()
#         plt.pause(0.001)
        
#         return True
    
#     def update_display_with_selection(self):
#         """Update display when a point is selected (without changing current data)."""
#         if self.current_frame is not None:
#             self.update_camera_display()
#             self.fig.canvas.draw_idle()
    
#     def update_camera_display(self):
#         """Update the camera image with overlay."""
#         if self.current_frame is None:
#             return
            
#         self.ax.clear()
#         self.ax.imshow(self.current_frame)
#         self.ax.set_title('Camera Feed with LiDAR Overlay')
        
#         # Overlay all LiDAR points on camera image
#         if self.current_uv_points.size > 0:
#             img_height, img_width = self.current_frame.shape[:2]
#             in_img = ((self.current_uv_points[:, 0] >= 0) & (self.current_uv_points[:, 0] < img_width) & 
#                      (self.current_uv_points[:, 1] >= 0) & (self.current_uv_points[:, 1] < img_height))
            
#             if np.any(in_img):
#                 # Plot regular points in red
#                 self.ax.scatter(self.current_uv_points[in_img, 0], self.current_uv_points[in_img, 1], 
#                                s=8, c='red', alpha=0.7, marker='o', label='LiDAR Points')
                
#                 # Highlight selected point in green with larger size if it exists and is visible
#                 if (self.selected_point_idx is not None and 
#                     self.selected_point_idx < len(in_img) and 
#                     in_img[self.selected_point_idx]):
                    
#                     selected_uv = self.current_uv_points[self.selected_point_idx]
#                     self.ax.scatter(selected_uv[0], selected_uv[1], 
#                                    s=100, c='lime', marker='x', linewidths=4,
#                                    label='Selected Point')
#                     # Add a circle around the selected point
#                     circle = plt.Circle((selected_uv[0], selected_uv[1]), 20, 
#                                       fill=False, color='lime', linewidth=3)
#                     self.ax.add_patch(circle)
        
#         self.ax.axis('off')
        
#         # Add legend
#         if self.current_uv_points.size > 0:
#             self.ax.legend(loc='upper right')
        
#         # Add status indicator
#         status_color = 'red' if self.paused else 'green'
#         status_text = 'PAUSED' if self.paused else 'PLAYING'
#         if self.zoom_enabled:
#             status_text += ' | ZOOM'
            
#         self.ax.text(0.02, 0.02, status_text, transform=self.ax.transAxes,
#                     bbox=dict(boxstyle='round', facecolor=status_color, alpha=0.7),
#                     color='white', fontweight='bold', fontsize=12)


# # ────────────────────────────────────────── Helper Functions ────────────────────────────

# def transform_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
#     """Convert ROS TransformStamped → 4×4 homogeneous matrix."""
#     t = tf_msg.transform.translation
#     q = tf_msg.transform.rotation
#     norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
#     if norm < 1e-6:
#         R = np.eye(3)
#         print("Warning: Invalid quaternion, using identity rotation")
#     else:
#         qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
#         R = np.array([
#             [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
#             [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
#             [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
#         ])
#     T = np.eye(4)
#     T[:3, :3] = R
#     T[:3, 3] = [t.x, t.y, t.z]
#     return T

# def project_points(points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
#     """Project 3‑D camera‑frame points → 2‑D pixels."""
#     in_front = points_cam[:, 2] > 1e-2
#     pts = points_cam[in_front]
#     if pts.size == 0:
#         return np.empty((0, 2))
#     uvw = K @ pts.T
#     uv = (uvw[:2] / uvw[2]).T
#     return uv

# def get_message_type_map() -> Dict[str, Any]:
#     return {
#         "/tf_static": TFMessage,
#         "/tf": TFMessage,
#         "/scan": LaserScan,
#         "/oakd/rgb/preview/image_raw": Image,
#         "/oakd/rgb/preview/camera_info": CameraInfo,
#     }

# def deserialize_message_safe(raw: bytes, msg_type: Any) -> Optional[Any]:
#     try:
#         return deserialize_message(raw, msg_type)
#     except Exception as exc:
#         print(f"Deserialization failed for {msg_type}: {exc}")
#         return None

# def open_bag_reader(path: Path) -> SequentialReader:
#     if path.is_file() and path.suffix == ".db3":
#         uri = str(path.parent)
#     else:
#         uri = str(path)
#     if not Path(uri).exists():
#         sys.exit(f"Bag path not found: {uri}")
#     reader = SequentialReader()
#     reader.open(StorageOptions(uri=uri, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
#     return reader

# def analyze_bag_topics(reader: SequentialReader) -> Dict[str, str]:
#     print("📄 Topics in bag:")
#     topics = {}
#     for meta in reader.get_all_topics_and_types():
#         topics[meta.name] = meta.type
#         print(f"  • {meta.name}  →  {meta.type}")
#     return topics

# def collect_static_data(reader: SequentialReader, type_map: Dict[str, Any], lidar_frame: str, camera_frame: str, cam_info_topic: str):
#     tf_buf = Buffer()
#     cam_info = None
#     processed = 0
#     while reader.has_next():
#         topic, raw, _ = reader.read_next()
#         processed += 1
#         msg = deserialize_message_safe(raw, type_map.get(topic)) if topic in type_map else None
#         if msg is None:
#             continue
#         if topic == "/tf_static":
#             for tf in msg.transforms:
#                 tf_buf.set_transform_static(tf, "bag")
#         elif topic == cam_info_topic and cam_info is None:
#             cam_info = msg
#         if cam_info is not None:
#             try:
#                 tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
#                 break
#             except LookupException:
#                 pass
#         if processed > 2000:
#             break
#     if cam_info is None:
#         sys.exit("❌ CameraInfo topic not found in bag")
#     try:
#         tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
#     except LookupException:
#         sys.exit(f"❌ No static TF from {lidar_frame} to {camera_frame}")
#     return tf_buf, cam_info

# # ────────────────────────────────────────── Main script ────────────────────────

# def parse_args():
#     ap = argparse.ArgumentParser(description="Interactive LiDAR overlay visualization - Camera View Only (ROS 2 Humble).")
#     ap.add_argument("bag", nargs="?", default="TestBag", type=Path, help="Bag directory (default ./TestBag)")
#     ap.add_argument("--lidar-frame", default="rplidar_link")
#     ap.add_argument("--camera-frame", default="oakd_rgb_camera_optical_frame")
#     ap.add_argument("--dt", type=float, default=0.05, help="Max |Δt| between RGB and scan (s)")
#     ap.add_argument("--scan-topic", default="/scan")
#     ap.add_argument("--image-topic", default="/oakd/rgb/preview/image_raw")
#     ap.add_argument("--camera-info-topic", default="/oakd/rgb/preview/camera_info")
#     ap.add_argument("--analyze", action="store_true", help="List topics and exit")
#     return ap.parse_args()

# def main():
#     args = parse_args()
#     bag_path = args.bag.expanduser().resolve()
#     print(f"▶️  Reading bag: {bag_path}")

#     rclpy.init()
#     bridge = CvBridge()

#     reader = open_bag_reader(bag_path)
#     available = analyze_bag_topics(reader)
#     if args.analyze:
#         return

#     type_map = get_message_type_map()
#     type_map[args.scan_topic] = LaserScan
#     type_map[args.image_topic] = Image
#     type_map[args.camera_info_topic] = CameraInfo

#     tf_buffer, cam_info = collect_static_data(reader, type_map, args.lidar_frame, args.camera_frame, args.camera_info_topic)
#     K = np.array(cam_info.k).reshape(3, 3)

#     # Initialize interactive visualizer
#     visualizer = InteractiveLiDARVisualizer(cam_info)

#     # Reopen for pass 2
#     reader = open_bag_reader(bag_path)
#     pending_scans: List[Tuple[float, LaserScan]] = []
#     processed = 0

#     print("\n🎮 Interactive Controls:")
#     print("  'p': Pause/Unpause video")
#     print("  'z': Enable/Disable zoom mode (only when paused)")
#     print("  'q': Quit")
#     print("  Space: Next frame (when paused)")
#     print("  Left click: Select LiDAR point (only when paused and zoom enabled)")
#     print("  When zoom enabled: Use matplotlib's toolbar for zoom/pan")
#     print()

#     try:
#         while reader.has_next() and not visualizer.quit_requested:
#             # Handle pause state
#             if visualizer.paused:
#                 plt.pause(0.1)
#                 continue
                
#             topic, raw, t_nsec = reader.read_next()
#             if topic not in type_map:
#                 continue
#             msg = deserialize_message_safe(raw, type_map[topic])
#             if msg is None:
#                 continue
#             t = t_nsec / 1e9
            
#             if topic == args.scan_topic:
#                 pending_scans.append((t, msg))
#                 pending_scans = pending_scans[-300:]
#                 continue
                
#             if topic != args.image_topic or not pending_scans:
#                 continue
                
#             times = np.array([s[0] for s in pending_scans])
#             idx = int(np.argmin(np.abs(times - t)))
#             if abs(times[idx] - t) > args.dt:
#                 continue
#             _, scan = pending_scans.pop(idx)

#             r = np.asarray(scan.ranges)
#             valid = np.isfinite(r) & (scan.range_min < r) & (r < scan.range_max)
#             if not np.any(valid):
#                 continue
                
#             a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
#             x_l = r[valid] * np.cos(a[valid])
#             y_l = r[valid] * np.sin(a[valid])
#             pts_l = np.column_stack((x_l, y_l, np.zeros_like(x_l), np.ones_like(x_l)))

#             T = transform_to_matrix(tf_buffer.lookup_transform(args.camera_frame, args.lidar_frame, TimeMsg()))
#             pts_c = (T @ pts_l.T).T[:, :3]
#             uv = project_points(pts_c, K).astype(int)

#             img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#             img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

#             processed += 1
            
#             # Update interactive visualization
#             if not visualizer.update_display(img_rgb, pts_c, uv):
#                 break
                
#             if processed % 50 == 0:
#                 print(f"🖼️  Processed {processed} frames")
                
#     except KeyboardInterrupt:
#         print("\n⚠️  Interrupted by user")
#     finally:
#         plt.close("all")
#         rclpy.shutdown()
#         print(f"🏁 Done. Total frames processed: {processed}")

# if __name__ == "__main__":
#     main()


#@@@@@@@@@@@@@@@@@@@
""""
Interactive LiDAR Point Cloud Visualizer with Two-Point Selection
Modified version with two-point selection and automatic range selection.

Workflow:
1. Press 'p' to pause video
2. Press 'z' to enable zoom mode (use matplotlib tools to zoom/pan)
3. Left click to select first LiDAR point
4. Optionally zoom/pan to find second point
5. Left click to select second LiDAR point
6. All points between the two selected points are automatically selected
7. when the zoom is enbled you can't select any point you should desbled it 

Controls:
- 'p': Pause/unpause the video stream
- 'z': Enable/disable zoom mode (only when paused)
- 'r': Reset selection (clear all selected points)
- Left click: Select LiDAR point (only when paused)
- 'q': Quit application
- Space: Step to next frame when paused
"""
from __future__ import annotations
import os
import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
import csv 
import datetime
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, TopicMetadata
from sensor_msgs.msg import CameraInfo, LaserScan, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, LookupException
import yaml 
from chessboard_calibration import ChessboardPlaneCalibrator
from types import MethodType
from types import MethodType
from chessboard_calibration import save_selection_to_file_chessboard
from chessboard_calibration import (       # NEW
    ChessboardPlaneCalibrator,
    save_selection_to_file_chessboard,
)
# ────────────── Interactive Visualizer Class ──────────────

class InteractiveLiDARVisualizer:
    def __init__(self, cam_info: CameraInfo,output_csv="lidar_selection_log.csv"):
        self.chessboard_calibrator = ChessboardPlaneCalibrator(
        chessboard_size=(7, 10),     # Update if needed
        square_size=0.025
        )
        self.save_selection_to_file = MethodType(
        save_selection_to_file_chessboard, self)       
        self.current_frame_number = 0  
        self.cam_info = cam_info
        self.paused = False
        self.quit_requested = False
        self.zoom_enabled = False
        self.current_frame = None
        self.current_lidar_points = None
        self.current_uv_points = None
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_csv = f"{output_csv}_{now}.csv"        # Write CSV header if file does not exist
        if not os.path.exists(self.output_csv):
            with open(self.output_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                #writer.writerow(['frame', 'lidar_index', 'distance', 'u', 'v'])
        self.save_selection_to_file = MethodType(save_selection_to_file_chessboard, self)
        # Selection state
        self.selection_step = 0
        self.first_selected_point = None
        self.first_selected_point_3d = None
        self.first_selected_idx = None
        self.second_selected_point = None
        self.second_selected_point_3d = None
        self.second_selected_idx = None
        self.selected_range_indices = []

        # Setup matplotlib figure
        self.fig, (self.ax_text, self.ax_img) = plt.subplots(1, 2, figsize=(14, 8), gridspec_kw={'width_ratios': [1, 2]})

        self.fig.suptitle('Interactive LiDAR Two-Point Selection Visualizer')
        self.ax_img.set_title('Camera Feed with LiDAR Overlay')
        self.ax_img.set_aspect('equal')
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        # self.info_text = self.fig.text(0.02, 0.98, '', transform=self.fig.transFigure,
        #                               verticalalignment='top', fontsize=10,
        #                               bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
        self.update_info_text()
        
        # Update this when you advance frames
    

    # def save_selection_to_file(self, indices, distances, uvs, frame_number):
    #     """Append selected 2D-3D correspondences to a CSV file in 'u,v,x,y,z' format."""
    #     with open(self.output_csv, 'a', newline='') as f:
    #         writer = csv.writer(f)
    #         for idx, dist, (u, v) in zip(indices, distances, uvs):
    #             angle = self.current_scan_angle_min + idx * self.current_scan_angle_increment
    #             x = dist * np.cos(angle)
    #             y = dist * np.sin(angle)
    #             z = 0.0  # 2D LiDAR assumption
    #             writer.writerow([u, v, x, y, z])


    # def save_selection_to_file(self, indices, distances, uvs, frame_number):
    #     """Append selected 2D-3D correspondences with plane parameters."""
    #     # Build 3D points
    #     angle_min = self.current_scan.angle_min
    #     angle_inc = self.current_scan.angle_increment
    #     points_3d = []
    #     for idx in indices:
    #         dist = self.current_ranges[idx]
    #         angle = angle_min + idx * angle_inc
    #         x = dist * np.cos(angle)
    #         y = dist * np.sin(angle)
    #         z = 0.0  # 2D LiDAR
    #         points_3d.append([x, y, z])
    #     points_3d = np.array(points_3d)

    #     # Fit plane using SVD
    #     centroid = np.mean(points_3d, axis=0)
    #     centered = points_3d - centroid
    #     _, _, vh = np.linalg.svd(centered)
    #     normal = vh[-1]
    #     normal /= np.linalg.norm(normal)
    #     a, b, c = normal
    #     d = -np.dot(normal, centroid)
    #     print(f"[DEBUG] Normal vector: a={a:.4f}, b={b:.4f}, c={c:.4f}, d={d:.4f}")

    #     # Save extended CSV
    #     with open(self.output_csv, 'a', newline='') as f:
    #         writer = csv.writer(f)
    #         for (idx, dist, (u, v), (x, y, z)) in zip(indices, distances, uvs, points_3d):
    #             writer.writerow([u, v, x, y, z, d, a, b, c])


   

    def update_info_text(self):
        lines = []

        # Header
        lines.append("==================================================")
        lines.append("          LIDAR-CAMERA INTERACTIVE STATUS         ")
        lines.append("==================================================")

        # Controls
        lines.append("\n[Controls]")
        lines.append("  p       → Pause / Play")
        lines.append("  z       → Toggle zoom mode (paused only)")
        lines.append("  r       → Reset selections")
        lines.append("  q       → Quit")
        lines.append("  Space   → Step to next frame (if paused)")

        # System Status
        lines.append("\n[System State]")
        lines.append(f"  Frame Number     : {self.current_frame_number}")
        lines.append(f"  Mode             : {'PAUSED' if self.paused else 'PLAYING'}")
        lines.append(f"  Zoom Enabled     : {'YES' if self.zoom_enabled else 'NO'}")

        # Selection Progress
        step_labels = [
            "Step 1: Pause and enable zoom",
            "Step 2: Select first LiDAR point",
            "Step 3: Select second LiDAR point",
            "Step 4: Range automatically selected"
        ]
        if not self.paused:
            current_step = 0
        elif not self.zoom_enabled and self.selection_step == 0:
            current_step = 0
        elif self.selection_step == 0:
            current_step = 1
        elif self.selection_step == 1:
            current_step = 2
        else:
            current_step = 3
        lines.append("\n[Selection Progress]")
        lines.append(f"  Current Step     : {step_labels[current_step]}")

        # First selected point
        if self.selection_step >= 1 and self.first_selected_point_3d is not None:
            x, y, z = self.first_selected_point_3d
            u, v = self.first_selected_point
            lines.append("\n[First Selected Point]")
            lines.append(f"  3D Coordinates   : (x={x:.3f}, y={y:.3f}, z={z:.3f})")
            lines.append(f"  2D Pixel         : (u={u:.0f}, v={v:.0f})")
            lines.append(f"  LiDAR Index      : {self.first_selected_idx}")

        # Second selected point
        if self.selection_step >= 2 and self.second_selected_point_3d is not None:
            x, y, z = self.second_selected_point_3d
            u, v = self.second_selected_point
            lines.append("\n[Second Selected Point]")
            lines.append(f"  3D Coordinates   : (x={x:.3f}, y={y:.3f}, z={z:.3f}) m")
            lines.append(f"  2D Pixel         : (u={u:.0f}, v={v:.0f})")
            lines.append(f"  LiDAR Index      : {self.second_selected_idx}")

        # Selected range
        if self.selection_step == 2 and len(self.selected_range_indices) > 0:
            lines.append("\n[Selected Range]")
            lines.append(f"  Points Selected  : {len(self.selected_range_indices)}")
            if self.current_ranges is not None:
                distances = [self.current_ranges[idx] for idx in self.selected_range_indices]
                lines.append(f"  Average Distance : {np.mean(distances):.2f} m")

        # CSV log path
        lines.append("\n[Logging]")
        lines.append(f"  Selection saved to:")
        lines.append(f"  {self.output_csv}")

        # Reminders
        if self.zoom_enabled:
            lines.append("\n[Note]")
            lines.append("  Zoom mode is currently ON.")
            lines.append("  Disable zoom (press 'z') to select points.")

        if not self.paused:
            lines.append("\n[Note]")
            lines.append("  Pause the video (press 'p') before selecting.")

        # Render to ax_text (with dark background)
        self.ax_text.clear()
        self.ax_text.set_facecolor("none")
        self.ax_text.axis("off")
        self.ax_text.text(
            0.01, 0.99,
            "\n".join(lines),
            va="top",
            ha="left",
            fontsize=10,
            family="monospace",
            color="black"
        )



    def on_key_press(self, event):
        if event.key == 'p':
            self.paused = not self.paused
            if not self.paused:
                self.zoom_enabled = False
                self.fig.canvas.toolbar.mode = ''
            self.update_info_text()
        elif event.key == 'z':
            if self.paused:
                self.zoom_enabled = not self.zoom_enabled
                if self.zoom_enabled:
                    print("Zoom mode enabled - Use matplotlib tools and click to select points")
                else:
                    self.fig.canvas.toolbar.mode = ''
                self.update_info_text()
            else:
                print("Zoom mode can only be toggled when video is paused")
        elif event.key == 'r':
            self.reset_selection()
        elif event.key == 'q':
            self.quit_requested = True
        elif event.key == ' ' and self.paused:
            pass

    def reset_selection(self):
        self.selection_step = 0
        self.first_selected_point = None
        self.first_selected_point_3d = None
        self.first_selected_idx = None
        self.second_selected_point = None
        self.second_selected_point_3d = None
        self.second_selected_idx = None
        self.selected_range_indices = []
        self.update_info_text()
        if self.current_frame is not None:
            self.update_display_with_selection()

    def on_mouse_click(self, event):
        if not self.paused or self.current_uv_points is None:
            return
        if self.zoom_enabled :
            print("desabled the zoom to select")
            return 
        if event.inaxes != self.ax_img:
            return
        if event.button == 1:
            if self.selection_step == 0:
                self.select_first_point(event.xdata, event.ydata)
            elif self.selection_step == 1:
                self.select_second_point(event.xdata, event.ydata)

    def select_first_point(self, click_x, click_y):
        selected_data = self.select_nearest_point(click_x, click_y)
        if selected_data is None:
            return
        uv_coord, point_3d, original_idx = selected_data
        self.first_selected_point = uv_coord
        self.first_selected_point_3d = point_3d
        self.first_selected_idx = original_idx
        self.selection_step = 1
        self.update_info_text()
        self.update_display_with_selection()
        if self.first_selected_idx is not None and self.current_ranges is not None:
            distance = self.current_ranges[self.first_selected_idx]
            uv = self.first_selected_point
            self.save_selection_to_file([self.first_selected_idx], [distance], [uv], self.current_frame_number)


    def select_second_point(self, click_x, click_y):
        selected_data = self.select_nearest_point(click_x, click_y)
        if selected_data is None:
            return
        uv_coord, point_3d, original_idx = selected_data
        self.second_selected_point = uv_coord
        self.second_selected_point_3d = point_3d
        self.second_selected_idx = original_idx
        self.selection_step = 2
        self.select_range_between_points()
        self.update_info_text()
        self.update_display_with_selection()

    def select_nearest_point(self, click_x, click_y):
        if self.current_uv_points is None or self.current_lidar_points is None or len(self.current_uv_points) == 0:
            return None
        if len(self.current_uv_points) != len(self.current_lidar_points):
            min_len = min(len(self.current_uv_points), len(self.current_lidar_points))
            uv_points = self.current_uv_points[:min_len]
            lidar_points = self.current_lidar_points[:min_len]
        else:
            uv_points = self.current_uv_points
            lidar_points = self.current_lidar_points
        img_height, img_width = self.current_frame.shape[:2]
        in_img = ((uv_points[:, 0] >= 0) & (uv_points[:, 0] < img_width) & (uv_points[:, 1] >= 0) & (uv_points[:, 1] < img_height))
        if not np.any(in_img):
            return None
        visible_uv = uv_points[in_img]
        visible_3d = lidar_points[in_img]
        distances = np.sqrt((visible_uv[:, 0] - click_x) ** 2 + (visible_uv[:, 1] - click_y) ** 2)
        nearest_idx = np.argmin(distances)
        uv_coord = (visible_uv[nearest_idx, 0], visible_uv[nearest_idx, 1])
        point_3d = (visible_3d[nearest_idx, 0], visible_3d[nearest_idx, 1], visible_3d[nearest_idx, 2])
        original_indices = np.where(in_img)[0]
        original_idx = original_indices[nearest_idx]
        return uv_coord, point_3d, original_idx

    def select_range_between_points(self):
        if (self.first_selected_point_3d is None or self.second_selected_point_3d is None or self.current_lidar_points is None):
            return
        p1 = np.array(self.first_selected_point_3d[:2])
        p2 = np.array(self.second_selected_point_3d[:2])
        points_2d = self.current_lidar_points[:, :2]
        line_vec = p2 - p1
        line_length = np.linalg.norm(line_vec)
        if line_length < 1e-6:
            self.selected_range_indices = [self.first_selected_idx, self.second_selected_idx]
            return
        line_unit = line_vec / line_length
        point_vecs = points_2d - p1
        projections = np.dot(point_vecs, line_unit)
        projected_points = p1 + projections[:, np.newaxis] * line_unit
        distances_to_line = np.linalg.norm(points_2d - projected_points, axis=1)
        on_segment = (projections >= 0) & (projections <= line_length)
        near_line = distances_to_line <= 0.5
        selected_mask = on_segment & near_line
        self.selected_range_indices = np.where(selected_mask)[0].tolist()
        if self.first_selected_idx not in self.selected_range_indices:
            self.selected_range_indices.append(self.first_selected_idx)
        if self.second_selected_idx not in self.selected_range_indices:
            self.selected_range_indices.append(self.second_selected_idx)
            
        if len(self.selected_range_indices) > 0 and self.current_ranges is not None:
            distances = [self.current_ranges[idx] for idx in self.selected_range_indices]
            uvs = [tuple(self.current_uv_points[idx]) for idx in self.selected_range_indices]
            self.save_selection_to_file(self.selected_range_indices, distances, uvs, self.current_frame_number)



    def update_display(self, img_rgb, lidar_points_cam, uv_points, lidar_ranges = None):
        if self.quit_requested:
            return False
        
        self.current_frame_number += 1  # Increment for each frame
        if lidar_ranges is not None:
            self.current_ranges = lidar_ranges  # Store current LiDAR distances

        self.current_frame = img_rgb
        self.current_lidar_points = lidar_points_cam
        self.current_uv_points = uv_points
        self.current_scan_angle_min = self.current_scan.angle_min
        self.current_scan_angle_increment = self.current_scan.angle_increment

        min_len = min(len(lidar_points_cam), len(uv_points))
        lidar_points_cam = lidar_points_cam[:min_len]
        uv_points = uv_points[:min_len]
        self.current_frame = img_rgb
        self.current_lidar_points = lidar_points_cam
        self.current_uv_points = uv_points
        self.update_camera_display()
        plt.tight_layout()
        self.fig.canvas.draw()
        plt.pause(0.001)
        return True


    # def save_selection_to_file(self, indices, distances, uvs, frame_number):
    #     """Append selected points to the CSV file."""
    #     with open(self.output_csv, 'a', newline='') as f:
    #         writer = csv.writer(f)
    #         for idx, dist, (u, v) in zip(indices, distances, uvs):
    #             writer.writerow([frame_number, idx, dist, u, v])


    def update_display_with_selection(self):
        if self.current_frame is not None:
            self.update_camera_display()
            self.fig.canvas.draw_idle()

   
    def update_camera_display(self):
        if self.current_frame is None:
            return

        self.ax_img.clear()
        self.ax_img.imshow(self.current_frame)
        self.ax_img.set_title('Camera Feed with LiDAR Overlay')

        if self.current_uv_points is not None and len(self.current_uv_points) > 0:
            img_height, img_width = self.current_frame.shape[:2]
            in_img = ((self.current_uv_points[:, 0] >= 0) & (self.current_uv_points[:, 0] < img_width) &
                    (self.current_uv_points[:, 1] >= 0) & (self.current_uv_points[:, 1] < img_height))

            if np.any(in_img):
                visible_uv = self.current_uv_points[in_img]
                self.ax_img.scatter(visible_uv[:, 0], visible_uv[:, 1],
                                    s=3, c='lime', alpha=0.5, marker='o', label='LiDAR Points')

                if len(self.selected_range_indices) > 0:
                    range_uv_points = []
                    for idx in self.selected_range_indices:
                        if idx < len(self.current_uv_points) and in_img[idx]:
                            range_uv_points.append(self.current_uv_points[idx])

                    if range_uv_points:
                        range_uv_points = np.array(range_uv_points)
                        self.ax_img.scatter(range_uv_points[:, 0], range_uv_points[:, 1],
                                            s=15, c='black', alpha=0.8, marker='.',
                                            label=f'Selected Range ({len(self.selected_range_indices)} pts)')

                if (self.first_selected_idx is not None and
                    self.first_selected_idx < len(self.current_uv_points) and
                    in_img[self.first_selected_idx]):
                    first_uv = self.current_uv_points[self.first_selected_idx]
                    self.ax_img.scatter(first_uv[0], first_uv[1],
                                        s=15, c='black', marker='x', linewidths=2)
                    circle1 = plt.Circle((first_uv[0], first_uv[1]), 10,
                                        fill=False, color='black', linewidth=1)
                    self.ax_img.add_patch(circle1)

                if (self.second_selected_idx is not None and
                    self.second_selected_idx < len(self.current_uv_points) and
                    in_img[self.second_selected_idx]):
                    second_uv = self.current_uv_points[self.second_selected_idx]
                    self.ax_img.scatter(second_uv[0], second_uv[1],
                                        s=15, c='black', marker='x', linewidths=2
                                        )
                    circle2 = plt.Circle((second_uv[0], second_uv[1]), 10,
                                        fill=False, color='black', linewidth=1)
                    self.ax_img.add_patch(circle2)

                if (self.first_selected_idx is not None and self.second_selected_idx is not None and
                    self.first_selected_idx < len(self.current_uv_points) and self.second_selected_idx < len(self.current_uv_points) and
                    in_img[self.first_selected_idx] and in_img[self.second_selected_idx]):
                    first_uv = self.current_uv_points[self.first_selected_idx]
                    second_uv = self.current_uv_points[self.second_selected_idx]
                    

        self.ax_img.axis('off')
        if self.current_uv_points is not None and len(self.current_uv_points) > 0:
            self.ax_img.legend(loc='upper right', fontsize=8)

# ────────────── Helper Functions ──────────────

def transform_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm < 1e-6:
        R = np.eye(3)
    else:
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
        ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T

def project_points(points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
    in_front = points_cam[:, 2] > 1e-2 # check if the Z is small or not than treshold (boolean)
    pts = points_cam[in_front]
    if pts.size == 0:
        return np.empty((0, 2)) # if there's no point -> quite
    uvw = K @ pts.T  # applies the camera intrinsic matrix to each point 
    uv = (uvw[:2] / uvw[2]).T #normalize the 2d projection by deviding by the third coordinate Z 
    return uv

def get_message_type_map() -> Dict[str, Any]:
    return {
        "/tf_static": TFMessage,
        "/tf": TFMessage,
        "/scan": LaserScan,
        "/oakd/rgb/preview/image_raw": Image,
        "/oakd/rgb/preview/camera_info": CameraInfo,
    }

def deserialize_message_safe(raw: bytes, msg_type: Any) -> Optional[Any]:
    try:
        return deserialize_message(raw, msg_type)
    except Exception as exc:
        print(f"Deserialization failed for {msg_type}: {exc}")
        return None

def save_intrinsics_and_extrinsics(K: np.ndarray, T_lidar_to_cam: np.ndarray, output_dir: str = "."):
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, f"extrinsic_intrinsic_{now}.txt")

        with open(filename, "w") as f:
            f.write("Camera Intrinsic Matrix (K):\n")
            for row in K:
                f.write("  " + "  ".join(f"{val:.6f}" for val in row) + "\n")

            f.write("\nExtrinsic Matrix (LiDAR to Camera - 4x4):\n")
            for row in T_lidar_to_cam:
                f.write("  " + "  ".join(f"{val:.6f}" for val in row) + "\n")

        print(f"[INFO] Intrinsic and extrinsic matrices saved to: {filename}")

def open_bag_reader(path: Path) -> SequentialReader:
    if path.is_file() and path.suffix == ".db3":
        uri = str(path.parent)
    else:
        uri = str(path)
    if not Path(uri).exists():
        sys.exit(f"Bag path not found: {uri}")
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    return reader

def analyze_bag_topics(reader: SequentialReader) -> Dict[str, str]:
    print("Topics in bag:")
    topics = {}
    for meta in reader.get_all_topics_and_types():
        topics[meta.name] = meta.type
        print(f"  • {meta.name}  →  {meta.type}")
    return topics

def collect_static_data(reader: SequentialReader, type_map: Dict[str, Any], lidar_frame: str, camera_frame: str, cam_info_topic: str):
    tf_buf = Buffer()
    cam_info = None
    processed = 0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        processed += 1
        msg = deserialize_message_safe(raw, type_map.get(topic)) if topic in type_map else None
        if msg is None:
            continue
        if topic == "/tf_static":
            for tf in msg.transforms:
                tf_buf.set_transform_static(tf, "bag")
        elif topic == cam_info_topic and cam_info is None:
            cam_info = msg
        if cam_info is not None:
            try:
                tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
                break
            except LookupException:
                pass
        if processed > 2000:
            break
    if cam_info is None:
        sys.exit("Error: CameraInfo topic not found in bag")
    try:
        tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
    except LookupException:
        sys.exit(f"Error: No static TF from {lidar_frame} to {camera_frame}")
    return tf_buf, cam_info

def parse_args():
    ap = argparse.ArgumentParser(description="Interactive LiDAR Two-Point Selection Visualizer (ROS 2 Humble).")
    ap.add_argument("bag", nargs="?", default="TestBag", type=Path, help="Bag directory (default ./TestBag)")
    ap.add_argument("--lidar-frame", default="rplidar_link")
    ap.add_argument("--camera-frame", default="oakd_rgb_camera_optical_frame")
    ap.add_argument("--dt", type=float, default=0.05, help="Max |Δt| between RGB and scan (s)")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--image-topic", default="/oakd/rgb/preview/image_raw")
    ap.add_argument("--camera-info-topic", default="/oakd/rgb/preview/camera_info")
    ap.add_argument("--analyze", action="store_true", help="List topics and exit")
    return ap.parse_args()

def main():
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    print(f"Reading bag: {bag_path}")

    rclpy.init()
    bridge = CvBridge()

    reader = open_bag_reader(bag_path)
    available = analyze_bag_topics(reader)  # print all the available topics
    if args.analyze:
        return

    type_map = get_message_type_map()  # get all topic name and also the msg type
    type_map[args.scan_topic] = LaserScan
    type_map[args.image_topic] = Image
    type_map[args.camera_info_topic] = CameraInfo

    # Extracts the static transforms "TF" and the camera's information
    tf_buffer, cam_info = collect_static_data(
        reader, type_map, args.lidar_frame, args.camera_frame, args.camera_info_topic
    )

    # Extracts the intrinsic matrix
    K = np.array(cam_info.k).reshape(3, 3)

    # Extract distortion coefficients
    dist_coeffs = np.array(cam_info.d).reshape(1, -1)

    # Save camera intrinsics to YAML
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    intrinsics_yaml_path = f"camera_intrinsics_{timestamp}.yaml"
    intrinsics_data = {
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": K.flatten().tolist()
        },
        "distortion_model": cam_info.distortion_model,
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(dist_coeffs.flatten()),
            "data": dist_coeffs.flatten().tolist()
        }
    }
    with open(intrinsics_yaml_path, 'w') as f:
        yaml.dump(intrinsics_data, f)
    print(f"[INFO] Camera intrinsics saved to {intrinsics_yaml_path}")

    # Initializes the matplotlib visualizer to allow the selection to display
    visualizer = InteractiveLiDARVisualizer(cam_info)

    reader = open_bag_reader(bag_path)  # reopen the bag
    pending_scans: List[Tuple[float, LaserScan]] = []  # buffer for recent LiDAR scans
    processed = 0  # frame counter

    try:
        while reader.has_next() and not visualizer.quit_requested:  # read the messages until the end or the user quits
            if visualizer.paused:  # wait if the visualizer is paused
                plt.pause(0.1)
                continue

            topic, raw, t_nsec = reader.read_next()  # the next message from the bag

            if topic not in type_map:  # skip irrelevant topics
                continue

            # Try to convert the raw binary message into a usable ROS 2 message
            msg = deserialize_message_safe(raw, type_map[topic])
            if msg is None:
                continue

            # Convert time to second
            t = t_nsec / 1e9

            if topic == args.scan_topic:  # if it is the topic scan
                pending_scans.append((t, msg))  # add it to the buffer
                pending_scans = pending_scans[-300:]  # keep just 300 messages
                continue

            if topic != args.image_topic or not pending_scans:  # focus only on LiDAR and image topics
                continue

            # Synchronization of the LiDAR data and images
            times = np.array([s[0] for s in pending_scans])  # Find the LiDAR scan closest in time to this image
            idx = int(np.argmin(np.abs(times - t)))  # return the best arg
            if abs(times[idx] - t) > args.dt:  # skip if they don't match
                continue

            # Get the matched scan and remove it from the buffer
            _, scan = pending_scans.pop(idx)

            # Convert scan to 3D point
            r = np.asarray(scan.ranges)
            valid = np.isfinite(r) & (scan.range_min < r) & (r < scan.range_max)  # filter the LiDAR points
            if not np.any(valid):
                continue

            a = scan.angle_min + np.arange(len(r)) * scan.angle_increment  # raw that contains the angle
            x_l = r[valid] * np.cos(a[valid])  # convert the polar coordinate into the cartesian coordinate
            y_l = r[valid] * np.sin(a[valid])
            pts_l = np.column_stack((x_l, y_l, np.zeros_like(x_l), np.ones_like(x_l)))  # add the homogeneous coordinates

            # Get the transformation matrix from the LiDAR to camera frame
            T = transform_to_matrix(tf_buffer.lookup_transform(args.camera_frame, args.lidar_frame, TimeMsg()))

            # Applies the transformation
            pts_c = (T @ pts_l.T).T[:, :3]

            # Filter the LiDAR points that are outside the camera frame
            in_front = pts_c[:, 2] > 1e-2

            # Use the intrinsic parameters to project the LiDAR point on the camera pixels frame
            uv = project_points(pts_c, K)

            # Convert ROS image to OpenCV RGB image
            img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            processed += 1

            # Save extrinsics only once
            if processed == 2:
                extrinsics_yaml_path = f"lidar_camera_extrinsics_{timestamp}.yaml"
                with open(extrinsics_yaml_path, 'w') as f:
                    yaml.dump({"extrinsic_matrix": T.tolist()}, f)
                print(f"[INFO] Extrinsic matrix saved to {extrinsics_yaml_path}")

            # Visualizer update
            
            visualizer.current_scan = scan  # Store scan for angle calculations
            visualizer.update_display(img_rgb, pts_c[in_front], uv, lidar_ranges=r[valid][in_front])


            if processed % 50 == 0:
                print(f"Processed {processed} frames")

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        plt.close("all")
        rclpy.shutdown()
        print(f"Done. Total frames processed: {processed}")

if __name__ == "__main__":
    main()
