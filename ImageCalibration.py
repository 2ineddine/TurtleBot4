###### the script displaying the video ######
# import cv2
# import numpy as np
# from pathlib import Path
# from rosbags.highlevel import AnyReader
# from rosbags.serde import deserialize_cdr
# from sensor_msgs.msg import Image

# # Path to your ROS 2 bag
# bag_path = Path('/media/zineddine/9D1D-BDBE/Turtlebot4/rosbag_17_06_all')

# # Function to display images as a video stream
# def display_video(image_data, encoding, width, height):
#     """Decode the image data and show it using OpenCV (as video)."""
#     if encoding == "rgb8":
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
#     elif encoding == "mono8":
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
#     elif encoding == "bgr8":  # Add support for bgr8 encoding
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
#     else:
#         raise ValueError(f"Unsupported encoding: {encoding}")

#     # Display the image
#     cv2.imshow("Camera Video", img_array)
    
#     # Wait for 1 ms, check for keypress to stop the video
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         return False  # Stop playback if 'q' is pressed
#     return True


# # --- Read bag and display video ---
# with AnyReader([bag_path]) as reader:
    
    
#     for connection, timestamp, rawdata in reader.messages():
#         if connection.topic == '/oakd/rgb/preview/image_raw':  # Change topic name if needed
#             # Deserialize the message
#             msg = deserialize_cdr(rawdata, connection.msgtype)
            
#             # Print out basic details about the image
#             print(f"Timestamp: {timestamp}")
#             print(f"Encoding: {msg.encoding}")
#             print(f"Width: {msg.width}, Height: {msg.height}")
            
#             # Display the image (video playback)
#             if not display_video(msg.data, msg.encoding, msg.width, msg.height):
#                 break  # Stop the video if 'q' is pressed

#     # Close the OpenCV window after finishing
#     cv2.destroyAllWindows()

##### the script for saving the image dedicated for the calibration #########
# import cv2
# import numpy as np
# from pathlib import Path
# from rosbags.highlevel import AnyReader
# from rosbags.serde import deserialize_cdr
# from sensor_msgs.msg import Image

# # Path to your ROS 2 bag
# bag_path = Path('/media/zineddine/9D1D-BDBE/Turtlebot4/rosbag_17_06_all')

# # Set the frame delay for video playback (in milliseconds)
# frame_delay = 30  # 30 ms delay for ~33 frames per second

# # Function to display images as a video stream
# def display_video(image_data, encoding, width, height):
#     """Decode the image data and show it using OpenCV (as video)."""
#     if encoding == "rgb8":
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
#     elif encoding == "mono8":
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
#     elif encoding == "bgr8":  # Add support for bgr8 encoding
#         img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
#     else:
#         raise ValueError(f"Unsupported encoding: {encoding}")

#     # Display the image
#     cv2.imshow("Camera Video", img_array)
    
#     return img_array

# # --- Read bag and display video ---
# with AnyReader([bag_path]) as reader:
    
    
#     for connection, timestamp, rawdata in reader.messages():
#         if connection.topic == '/oakd/rgb/preview/image_raw':  # Change topic name if needed
#             # Deserialize the message
#             msg = deserialize_cdr(rawdata, connection.msgtype)
            
#             # Print out basic details about the image
#             print(f"Timestamp: {timestamp}")
#             print(f"Encoding: {msg.encoding}")
#             print(f"Width: {msg.width}, Height: {msg.height}")
            
#             # Display the image (video playback)
#             img_array = display_video(msg.data, msg.encoding, msg.width, msg.height)

#             # Wait for a keypress and control the video speed using waitKey
#             key = cv2.waitKey(frame_delay) & 0xFF  # Adjust frame delay to control speed
#             if key == ord('s'):  # If 's' is pressed, save the current image
#                 timestamp_str = str(timestamp)
#                 filename = f"saved_image_{timestamp_str}.png"
#                 cv2.imwrite(filename, img_array)
#                 print(f"Image saved as {filename}")

#             elif key == ord('q'):  # If 'q' is pressed, quit
#                 break  # Stop the video if 'q' is pressed

#     # Close the OpenCV window after finishing
#     cv2.destroyAllWindows()
###### the script dedicated to calibrate the camera by openCV ######
# import cv2
# import numpy as np
# import os
# import glob
 
# # Defining the dimensions of checkerboard
# CHECKERBOARD = (7,10)
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# # Creating vector to store vectors of 3D points for each checkerboard image
# objpoints = []
# # Creating vector to store vectors of 2D points for each checkerboard image
# imgpoints = [] 
 
 
# # Defining the world coordinates for 3D points
# objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
# prev_img_shape = None
 
# # Extracting path of individual image stored in a given directory
# images = glob.glob('./*.png')
# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     # If desired number of corners are found in the image then ret = true
#     ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
     
#     """
#     If desired number of corner are detected,
#     we refine the pixel coordinates and display 
#     them on the images of checker board
#     """
#     if ret == True:
#         objpoints.append(objp)
#         # refining pixel coordinates for given 2d points.
#         corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
         
#         imgpoints.append(corners2)
 
#         # Draw and display the corners
#         img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
     
#     cv2.imshow('img',img)
#     cv2.waitKey(0)
 
# cv2.destroyAllWindows()
 
# h,w = img.shape[:2]
 
# """
# Performing camera calibration by 
# passing the value of known 3D points (objpoints)
# and corresponding pixel coordinates of the 
# detected corners (imgpoints)
# """
# ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
 
# print("Camera matrix : \n")
# print(mtx)
# print("dist : \n")
# print(dist)
# print("rvecs : \n")
# print(rvecs)
# print("tvecs : \n")
# print(tvecs)


##### chosing the relevant LiDAR's points for the calibration ####################
#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
# """
# Live LiDAR-stripe picker   (ROS 2 • rosbag2 • pure Python)

# Keys
#   p        pause / resume playback
#   ENTER    save selected stripe → segment_indices_<frame>.yaml
#   - / +    slower / faster playback
#   s        skip current frame (while paused)
#   q        quit
# Mouse
#   Pause first (p), then left-click first end-point, left-click second
#   All beams between the two angles turn red.

# Dependencies:
#   pip install --user rosbags numpy matplotlib pyyaml
# """

# # ───────── USER SETTINGS ────────────────────────────────────────────
# BAG_PATH       = "./rosbag_17_06_all"   # folder with metadata.yaml + *.db3
# SCAN_TOPIC     = "/scan"                # change if your topic differs
# INIT_DELAY_MS  = 30                     # 30 ms ≈ 33 fps
# DELAY_MIN_MS   = 5
# DELAY_MAX_MS   = 2000
# # ────────────────────────────────────────────────────────────────────

# from pathlib import Path
# import yaml, numpy as np
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# from matplotlib.widgets import RectangleSelector
# from rosbags.highlevel import AnyReader

# # ───────── open bag + preload scans ─────────────────────────────────
# # Set up typestore for ROS 2 Humble to avoid deprecation warning
# try:
#     from rosbags.typesys.stores.humble import get_typestore
#     typestore = get_typestore()
# except ImportError:
#     # Fallback if humble typestore not available
#     typestore = None

# reader = AnyReader([Path(BAG_PATH)], default_typestore=typestore)
# reader.open()
# con_scan = next(c for c in reader.connections if c.topic == SCAN_TOPIC)

# scans = []
# for _, _, raw in reader.messages(connections=[con_scan]):
#     # Use proper rosbags deserialization instead of manual unpacking
#     msg = reader.deserialize(raw, con_scan.msgtype)
#     ranges = np.array(msg.ranges)
#     scans.append({
#         'angle_min': msg.angle_min, 
#         'angle_max': msg.angle_max, 
#         'ranges': ranges
#     })

# N = len(scans)
# assert N, "bag contains no /scan messages"
# print(f"Loaded {N} scan frames from {BAG_PATH}")

# # ───────── helpers ─────────────────────────────────────────────────
# def load_scan(k):
#     sc  = scans[k]
#     ang = np.linspace(sc['angle_min'], sc['angle_max'], len(sc['ranges']))
#     rng = sc['ranges']
#     return np.vstack([np.degrees(ang), rng]).T, ang, rng

# def save_yaml(mask, ang, idx):
#     fn = f"segment_indices_{idx}.yaml"
#     sel = np.where(mask)[0].tolist()
#     yaml.safe_dump({'frame': idx,
#                     'indices': sel,
#                     'angles':  [float(ang[i]) for i in sel]},
#                    open(fn, 'w'))
#     print(f"✓ frame {idx}: saved {len(sel)} beams → {fn}")

# # ───────── define callback function first ─────────────────────────────
# def onselect(eclick, erelease):
#     global sel_mask
#     x0, y0 = eclick.xdata,  eclick.ydata
#     x1, y1 = erelease.xdata, erelease.ydata
#     xmin, xmax = sorted([x0, x1]); ymin, ymax = sorted([y0, y1])
#     sel_mask = ((pts[:, 0] >= xmin) & (pts[:, 0] <= xmax) &
#                 (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax))
#     scatter.set_color(np.where(sel_mask, 'r', 'b'))
#     fig.canvas.draw_idle()

# # ───────── GUI initialisation ──────────────────────────────────────
# mpl.use("TkAgg")
# fig, ax = plt.subplots(figsize=(8, 4))
# ax.set_xlabel("angle (deg)")
# ax.set_ylabel("range (m)")
# # North / South markers
# ax.axvline(0,   color='gray', lw=1, ls='--')
# ax.axvline(180, color='gray', lw=1, ls='--')
# ax.text(  2, 0.95, 'N', transform=ax.transAxes, color='gray')
# ax.text(0.98, 0.95, 'S', transform=ax.transAxes, color='gray',
#         ha='right')

# frame = 0
# running = True
# delay_ms = INIT_DELAY_MS
# sel_mask = np.zeros(len(scans[0]['ranges']), bool)

# pts, ang_rad, rng = load_scan(frame)
# scatter = ax.scatter(pts[:, 0], pts[:, 1], s=6, c='b')
# ax.set_title("p pause/resume   ENTER save   -/+ slower/faster   q quit")
# plt.tight_layout()

# rect = RectangleSelector(ax, onselect, interactive=False, useblit=True,
#                          minspanx=1, minspany=1, button=[1])
# rect.set_visible(False)

# def on_key(event):
#     global running, frame, sel_mask, pts, ang_rad, rng, delay_ms
#     if event.key == 'p':
#         running = not running
#         rect.set_visible(not running)
#     elif event.key == 'enter' and not running:
#         if sel_mask.any():
#             save_yaml(sel_mask, ang_rad, frame)
#         else:
#             print("⚠ nothing selected")
#         sel_mask[:] = False
#         running = True
#         rect.set_visible(False)
#     elif event.key == 's' and not running:
#         running = True
#         rect.set_visible(False)
#     elif event.key == '+':
#         delay_ms = max(DELAY_MIN_MS, delay_ms // 2)
#         timer.stop(); timer.interval = delay_ms; timer.start()
#         print(f"speed ↑  ({1000/delay_ms:.1f} fps)")
#     elif event.key == '-':
#         delay_ms = min(DELAY_MAX_MS, delay_ms * 2)
#         timer.stop(); timer.interval = delay_ms; timer.start()
#         print(f"speed ↓  ({1000/delay_ms:.1f} fps)")
#     elif event.key == 'q':
#         plt.close(fig)

# fig.canvas.mpl_connect('key_press_event', on_key)

# def update(_):
#     global frame, pts, ang_rad, rng, sel_mask
#     if not running:
#         return
#     frame += 1
#     if frame >= N:
#         print("End of bag"); plt.close(fig); return
#     pts, ang_rad, rng = load_scan(frame)
#     sel_mask = np.zeros(len(rng), bool)
#     scatter.set_offsets(pts)
#     scatter.set_color('b')
#     ax.set_title(f"Frame {frame}/{N-1}   (p pause)")
#     fig.canvas.draw_idle()

# timer = fig.canvas.new_timer(interval=delay_ms)
# timer.add_callback(update, None)
# timer.start()
# plt.show()


############# displaying just the front of the Lidar ##################
# """
# Live LiDAR Point Cloud Picker   (ROS 2 • rosbag2 • pure Python)
# RViz-style 2D point cloud visualization

# Keys
#   p        pause / resume playback
#   ENTER    save selected points → segment_indices_<frame>.yaml
#   - / +    slower / faster playback
#   s        skip current frame (while paused)
#   q        quit
# Mouse
#   Pause first (p), then left-click and drag to select rectangular area
#   All points in the rectangle turn red.

# Dependencies:
#   pip install --user rosbags numpy matplotlib pyyaml
# """

# # ───────── USER SETTINGS ────────────────────────────────────────────
# BAG_PATH       = "./rosbag_17_06_all"   # folder with metadata.yaml + *.db3
# SCAN_TOPIC     = "/scan"                # change if your topic differs
# INIT_DELAY_MS  = 30                     # 30 ms ≈ 33 fps
# DELAY_MIN_MS   = 5
# DELAY_MAX_MS   = 2000
# MAX_RANGE      = 30.0                   # Max display range in meters
# # ────────────────────────────────────────────────────────────────────

# from pathlib import Path
# import yaml, numpy as np
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# from matplotlib.widgets import RectangleSelector
# from rosbags.highlevel import AnyReader

# # ───────── open bag + preload scans ─────────────────────────────────
# # Set up typestore for ROS 2 Humble to avoid deprecation warning
# try:
#     from rosbags.typesys.stores.humble import get_typestore
#     typestore = get_typestore()
# except ImportError:
#     # Fallback if humble typestore not available
#     typestore = None

# reader = AnyReader([Path(BAG_PATH)], default_typestore=typestore)
# reader.open()
# con_scan = next(c for c in reader.connections if c.topic == SCAN_TOPIC)

# scans = []
# for _, _, raw in reader.messages(connections=[con_scan]):
#     # Use proper rosbags deserialization instead of manual unpacking
#     msg = reader.deserialize(raw, con_scan.msgtype)
#     ranges = np.array(msg.ranges)
#     scans.append({
#         'angle_min': msg.angle_min, 
#         'angle_max': msg.angle_max, 
#         'ranges': ranges
#     })

# N = len(scans)
# assert N, "bag contains no /scan messages"
# print(f"Loaded {N} scan frames from {BAG_PATH}")

# # ───────── helpers ─────────────────────────────────────────────────
# def load_scan_cartesian(k):
#     """Convert polar LiDAR data to Cartesian coordinates (X, Y)"""
#     sc = scans[k]
#     angles = np.linspace(sc['angle_min'], sc['angle_max'], len(sc['ranges']))
#     ranges = sc['ranges']
    
#     # Filter out invalid ranges (inf, nan, too far)
#     valid_mask = np.isfinite(ranges) & (ranges > 0) & (ranges < MAX_RANGE)
#     valid_angles = angles[valid_mask]
#     valid_ranges = ranges[valid_mask]
    
#     # Convert to Cartesian coordinates
#     # X = forward, Y = left (RViz convention)
#     x = valid_ranges * np.cos(valid_angles)
#     y = valid_ranges * np.sin(valid_angles)
    
#     return np.column_stack([x, y]), valid_angles, valid_ranges, valid_mask

# def save_yaml(mask, ang, idx, original_mask):
#     """Save selected points indices"""
#     fn = f"segment_indices_{idx}.yaml"
#     # Map back to original scan indices
#     original_indices = np.where(original_mask)[0]
#     selected_original = original_indices[mask]
    
#     yaml.safe_dump({
#         'frame': idx,
#         'indices': selected_original.tolist(),
#         'angles': [float(ang[i]) for i in range(len(ang)) if mask[i]]
#     }, open(fn, 'w'))
#     print(f"✓ frame {idx}: saved {np.sum(mask)} points → {fn}")

# # ───────── define callback function first ─────────────────────────────
# def onselect(eclick, erelease):
#     global sel_mask
#     x0, y0 = eclick.xdata,  eclick.ydata
#     x1, y1 = erelease.xdata, erelease.ydata
#     xmin, xmax = sorted([x0, x1]); ymin, ymax = sorted([y0, y1])
#     sel_mask = ((pts[:, 0] >= xmin) & (pts[:, 0] <= xmax) &
#                 (pts[:, 1] >= ymin) & (pts[:, 1] <= ymax))
#     colors = np.where(sel_mask, 'red', 'blue')
#     scatter.set_color(colors)
#     fig.canvas.draw_idle()
#     print(f"Selected {np.sum(sel_mask)} points")

# # ───────── GUI initialisation ──────────────────────────────────────
# mpl.use("TkAgg")
# fig, ax = plt.subplots(figsize=(10, 10))
# ax.set_xlabel("X (meters) - Forward →")
# ax.set_ylabel("Y (meters) - Left →")
# ax.set_aspect('equal')

# # Draw coordinate system indicators
# ax.axhline(0, color='gray', lw=0.5, alpha=0.7)
# ax.axvline(0, color='gray', lw=0.5, alpha=0.7)

# # Draw range circles for reference
# for r in [5, 10, 15, 20, 25]:
#     if r <= MAX_RANGE:
#         circle = plt.Circle((0, 0), r, fill=False, color='lightgray', 
#                           linestyle='--', alpha=0.3, linewidth=0.5)
#         ax.add_patch(circle)
#         ax.text(r, 0.5, f'{r}m', fontsize=8, color='gray', alpha=0.7)

# # Highlight the front 180° arc area (forward hemisphere)
# theta = np.linspace(-np.pi/2, np.pi/2, 100)
# front_arc_x = MAX_RANGE * np.cos(theta)
# front_arc_y = MAX_RANGE * np.sin(theta)
# ax.fill_between(front_arc_x, front_arc_y, 0, alpha=0.05, color='red', 
#                 label='Front 180° Arc')

# # Add directional arrows and labels
# ax.arrow(0, 0, 3, 0, head_width=0.5, head_length=0.3, fc='red', ec='red', alpha=0.8)
# ax.text(4, 0.5, 'FORWARD', fontweight='bold', color='red', fontsize=10)
# ax.arrow(0, 0, 0, 3, head_width=0.5, head_length=0.3, fc='green', ec='green', alpha=0.8)
# ax.text(0.5, 4, 'LEFT', fontweight='bold', color='green', fontsize=10)

# # LiDAR sensor position indicator
# ax.plot(0, 0, 'ko', markersize=8, label='LiDAR Sensor')

# ax.set_xlim(-MAX_RANGE, MAX_RANGE)
# ax.set_ylim(-MAX_RANGE, MAX_RANGE)
# ax.grid(True, alpha=0.3)
# ax.legend(loc='upper right')

# frame = 0
# running = True
# delay_ms = INIT_DELAY_MS
# sel_mask = None

# # Load initial frame
# pts, ang_rad, rng, original_mask = load_scan_cartesian(frame)
# sel_mask = np.zeros(len(pts), bool)
# scatter = ax.scatter(pts[:, 0], pts[:, 1], s=3, c='blue', alpha=0.7)

# ax.set_title("🌐 LiDAR Point Cloud View | p=pause | ENTER=save | +/-=speed | q=quit")
# plt.tight_layout()

# rect = RectangleSelector(ax, onselect, interactive=False, useblit=False,
#                          minspanx=0.1, minspany=0.1, button=[1])
# rect.set_visible(False)

# def on_key(event):
#     global running, frame, sel_mask, pts, ang_rad, rng, delay_ms, original_mask
#     if event.key == 'p':
#         running = not running
#         rect.set_visible(not running)
#         if not running:
#             print("⏸ PAUSED - Click and drag to select area in point cloud")
#         else:
#             print("▶ RESUMED")
#     elif event.key == 'enter' and not running:
#         if sel_mask is not None and sel_mask.any():
#             save_yaml(sel_mask, ang_rad, frame, original_mask)
#         else:
#             print("⚠ nothing selected")
#         if sel_mask is not None:
#             sel_mask[:] = False
#         running = True
#         rect.set_visible(False)
#     elif event.key == 's' and not running:
#         print(f"⏭ Skipping frame {frame}")
#         running = True
#         rect.set_visible(False)
#     elif event.key == '+':
#         delay_ms = max(DELAY_MIN_MS, delay_ms // 2)
#         timer.stop(); timer.interval = delay_ms; timer.start()
#         print(f"⚡ Speed ↑  ({1000/delay_ms:.1f} fps)")
#     elif event.key == '-':
#         delay_ms = min(DELAY_MAX_MS, delay_ms * 2)
#         timer.stop(); timer.interval = delay_ms; timer.start()
#         print(f"🐌 Speed ↓  ({1000/delay_ms:.1f} fps)")
#     elif event.key == 'q':
#         print("👋 Goodbye!")
#         plt.close(fig)

# fig.canvas.mpl_connect('key_press_event', on_key)

# def update(_):
#     global frame, pts, ang_rad, rng, sel_mask, original_mask
#     if not running:
#         return
#     frame += 1
#     if frame >= N:
#         print("🏁 End of bag"); plt.close(fig); return
    
#     pts, ang_rad, rng, original_mask = load_scan_cartesian(frame)
#     sel_mask = np.zeros(len(pts), bool)
    
#     if len(pts) > 0:
#         scatter.set_offsets(pts)
#         scatter.set_color('blue')
#         scatter.set_sizes([3] * len(pts))
    
#     ax.set_title(f"Frame {frame}/{N-1} | 📡 {len(pts)} points | (p to pause)")
#     fig.canvas.draw_idle()

# # Print initial info
# print(f"🚀 Starting LiDAR Point Cloud Picker...")
# print(f"📊 Loaded {N} frames from {SCAN_TOPIC}")
# print(f"🌐 RViz-style 2D point cloud visualization")
# print(f"🔴 RED SHADED AREA = FRONT 180° ARC")
# print(f"⌨️  Controls: p=pause, ENTER=save, +/-=speed, q=quit")
# print(f"🖱️  Mouse: Drag rectangle to select points (while paused)")

# timer = fig.canvas.new_timer(interval=delay_ms)
# timer.add_callback(update, None)
# timer.start()
# plt.show()


###### second test to display the lidar point on the camera ################
#!/usr/bin/env python3
#!/usr/bin/env python3
# """
# interactive_overlay.py  –  Live viewer that overlays LiDAR rays on Oak-D RGB frames.

# Key-bindings while the window is open
# -------------------------------------
#   q / ESC    Quit immediately
#   SPACE      Pause / resume (toggles)
#   n / RIGHT  Step one scan forward when paused
# """

# from pathlib import Path
# import cv2, numpy as np, rclpy, tf2_ros, tf_transformations
# from cv_bridge import CvBridge
# from sensor_msgs.msg import LaserScan, CameraInfo
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
# from builtin_interfaces.msg import Time
# from rosbags.highlevel import AnyReader
# from collections import defaultdict
# import bisect


# # -------- EDIT ONLY THESE THREE PATHS / TOPICS --------------------------------
# # For ROS 2 bag with .db3 files, use the folder path, not individual .db3 files
# BAG_FILE   = Path('./rosbag_17_06_all')  # Folder containing metadata.yaml and .db3 files
# IMG_TOPIC  = '/oakd/rgb/preview/image_raw'
# INFO_TOPIC = '/oakd/rgb/preview/camera_info'
# SCAN_TOPIC = '/scan'
# CAM_FRAME  = 'oakd_rgb_camera_optical_frame'        # found with view_frames / topic echo
# # -----------------------------------------------------------------------------

# # ---- helper: LaserScan ➜ xyz in the laser frame -----------------------------
# def scan_to_xyz(scan: LaserScan) -> np.ndarray:
#     angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
#     ranges = np.asarray(scan.ranges, dtype=np.float32)
#     valid  = np.isfinite(ranges)
#     ranges, angles = ranges[valid], angles[valid]
#     x = ranges * np.cos(angles)
#     y = ranges * np.sin(angles)
#     z = np.zeros_like(x)
#     return np.vstack((x, y, z))          # shape (3,N)

# # ---- helper: apply geometry_msgs/TransformStamped to xyz --------------------
# def transform_xyz(T, xyz):
#     q = T.transform.rotation
#     t = T.transform.translation
#     R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
#     return R @ xyz + np.asarray([[t.x], [t.y], [t.z]])

# # ---- helper: convert timestamp to nanoseconds -------------------------------
# def timestamp_to_ns(stamp):
#     """Convert various timestamp formats to nanoseconds"""
#     if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
#         return stamp.sec * 1_000_000_000 + stamp.nanosec
#     elif hasattr(stamp, 'secs') and hasattr(stamp, 'nsecs'):
#         return stamp.secs * 1_000_000_000 + stamp.nsecs
#     else:
#         # Fallback for other timestamp formats
#         return int(stamp) if isinstance(stamp, (int, float)) else 0

# # ---- helper: find closest message by timestamp ------------------------------
# def find_closest_message(messages_with_time, target_time_ns, max_time_diff_ns=100_000_000):  # 100ms
#     """Find the message closest in time to target_time_ns"""
#     if not messages_with_time:
#         return None
    
#     times = [t for t, _ in messages_with_time]
#     idx = bisect.bisect_left(times, target_time_ns)
    
#     candidates = []
#     if idx < len(times):
#         candidates.append((abs(times[idx] - target_time_ns), messages_with_time[idx]))
#     if idx > 0:
#         candidates.append((abs(times[idx-1] - target_time_ns), messages_with_time[idx-1]))
    
#     if candidates:
#         time_diff, (_, message) = min(candidates)
#         if time_diff <= max_time_diff_ns:
#             return message
    
#     return None

# # -----------------------------------------------------------------------------

# def main():
#     # ── ROS infrastructure just for tf2 --------------------------------------
#     rclpy.init()
#     node      = rclpy.create_node('interactive_overlay')
#     tf_buffer = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer, node)        # no live data
#     bridge    = CvBridge()

#     # ── read the bag once, all on-disk streaming -----------------------------
#     with AnyReader([BAG_FILE]) as reader:
#         cons = {c.topic: c for c in reader.connections}

#         # Camera intrinsics K
#         info_raw = next(reader.messages(connections=[cons[INFO_TOPIC]]))[2]
#         info_msg: CameraInfo = reader.deserialize(info_raw, cons[INFO_TOPIC].msgtype)
#         K = np.asarray(info_msg.k, dtype=np.float32).reshape(3, 3)

#         # Pre-load every TF transform into the buffer
#         print("[tf] Loading transforms from bag file...")
#         transform_count = 0
#         for c in reader.connections:
#             if c.topic in ('/tf', '/tf_static'):
#                 for _, _, raw in reader.messages(connections=[c]):
#                     tf_msg: TFMessage = reader.deserialize(raw, c.msgtype)
#                     for tr in tf_msg.transforms:
#                         try:
#                             # Convert rosbags transform to proper ROS message type
#                             ros_transform = TransformStamped()
                            
#                             # Convert timestamp properly
#                             if hasattr(tr.header.stamp, 'sec') and hasattr(tr.header.stamp, 'nanosec'):
#                                 # ROS 2 style timestamp
#                                 ros_transform.header.stamp.sec = int(tr.header.stamp.sec)
#                                 ros_transform.header.stamp.nanosec = int(tr.header.stamp.nanosec)
#                             elif hasattr(tr.header.stamp, 'secs') and hasattr(tr.header.stamp, 'nsecs'):
#                                 # ROS 1 style timestamp
#                                 ros_transform.header.stamp.sec = int(tr.header.stamp.secs)
#                                 ros_transform.header.stamp.nanosec = int(tr.header.stamp.nsecs)
#                             else:
#                                 # Try to extract timestamp from raw value
#                                 stamp_val = tr.header.stamp
#                                 if hasattr(stamp_val, '__dict__'):
#                                     # Debug first few timestamps
#                                     if transform_count < 3:
#                                         print(f"[tf] Debug stamp: {stamp_val}, type: {type(stamp_val)}, attrs: {dir(stamp_val)}")
#                                 ros_transform.header.stamp = Time()
#                                 ros_transform.header.stamp.sec = 0
#                                 ros_transform.header.stamp.nanosec = 0
                            
#                             ros_transform.header.frame_id = str(tr.header.frame_id)
#                             ros_transform.child_frame_id = str(tr.child_frame_id)
                            
#                             # Convert translation
#                             ros_transform.transform.translation.x = float(tr.transform.translation.x)
#                             ros_transform.transform.translation.y = float(tr.transform.translation.y) 
#                             ros_transform.transform.translation.z = float(tr.transform.translation.z)
                            
#                             # Convert rotation
#                             ros_transform.transform.rotation.x = float(tr.transform.rotation.x)
#                             ros_transform.transform.rotation.y = float(tr.transform.rotation.y)
#                             ros_transform.transform.rotation.z = float(tr.transform.rotation.z)
#                             ros_transform.transform.rotation.w = float(tr.transform.rotation.w)
                            
#                             tf_buffer.set_transform(ros_transform, 'bag')
#                             transform_count += 1
                            
#                             if transform_count % 100 == 0:  # Progress indicator
#                                 print(f"[tf] Loaded {transform_count} transforms...")
                                
#                         except Exception as e:
#                             if transform_count < 5:  # Only show first few errors to avoid spam
#                                 print(f"[tf] Failed to convert transform {tr.header.frame_id} -> {tr.child_frame_id}: {e}")
#                             continue
        
#         print(f"[tf] Loaded {transform_count} transforms successfully")

#         # ── Load and sort all messages by timestamp for synchronization --------
#         print("[data] Loading and sorting messages...")
        
#         # Load image messages
#         image_messages = []
#         if IMG_TOPIC in cons:
#             for _, _, raw in reader.messages(connections=[cons[IMG_TOPIC]]):
#                 img_msg = reader.deserialize(raw, cons[IMG_TOPIC].msgtype)
#                 timestamp_ns = timestamp_to_ns(img_msg.header.stamp)
#                 image_messages.append((timestamp_ns, img_msg))
        
#         # Load scan messages  
#         scan_messages = []
#         if SCAN_TOPIC in cons:
#             for _, _, raw in reader.messages(connections=[cons[SCAN_TOPIC]]):
#                 scan_msg = reader.deserialize(raw, cons[SCAN_TOPIC].msgtype)
#                 timestamp_ns = timestamp_to_ns(scan_msg.header.stamp)
#                 scan_messages.append((timestamp_ns, scan_msg))
        
#         # Sort messages by timestamp
#         image_messages.sort(key=lambda x: x[0])
#         scan_messages.sort(key=lambda x: x[0])
        
#         print(f"[data] Loaded {len(image_messages)} image messages and {len(scan_messages)} scan messages")
        
#         if not image_messages or not scan_messages:
#             print("[error] No messages found for synchronization")
#             return

#         # ── OpenCV window set-up and processing loop ----------------------------
#         paused = False
#         current_scan_idx = 0
#         cv2.namedWindow('overlay', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        
#         def process_frame(scan_msg, img_msg):
#             """Process the current frame with LiDAR scan data"""
#             ts = scan_msg.header.stamp
#             print(f"[overlay] Processing scan at time: {timestamp_to_ns(ts)}")

#             try:
#                 # Use the timestamp from the scan message for transform lookup
#                 time_obj = rclpy.time.Time(nanoseconds=timestamp_to_ns(ts))
#                 T = tf_buffer.lookup_transform(CAM_FRAME, scan_msg.header.frame_id, time_obj)
#             except tf2_ros.ExtrapolationException as e:
#                 print(f"[tf] Lookup failed: {e}")
#                 return False  # Skip if transform lookup fails

#             # From scan to camera xyz
#             xyz_laser = scan_to_xyz(scan_msg)  # (3,N)
#             xyz_cam = transform_xyz(T, xyz_laser)  # (3,N)

#             # Keep points in front of camera
#             in_front = xyz_cam[2] > 0.01
#             if not in_front.any():
#                 print("[overlay] No points in front of camera")
#                 return True
#             xyz_cam = xyz_cam[:, in_front]

#             # Perspective projection
#             uv = (K @ xyz_cam) / xyz_cam[2]
#             uv = uv[:2].T.round().astype(int)  # (N,2) ints

#             frame = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
#             H, W, _ = frame.shape
#             valid = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
            
#             # Draw LiDAR points as red pixels
#             for u, v in uv[valid]:
#                 frame[v, u] = (0, 0, 255)  # red pixel

#             # Add status text
#             status_text = f"Scan {current_scan_idx + 1}/{len(scan_messages)}"
#             if paused:
#                 status_text += " [PAUSED]"
#             cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
#             cv2.imshow('overlay', frame)
#             return True

#         # Main processing loop
#         while current_scan_idx < len(scan_messages):
#             if not paused or current_scan_idx == 0:  # Always process first frame
#                 # Get current scan message
#                 scan_timestamp_ns, scan_msg = scan_messages[current_scan_idx]
                
#                 # Find closest image message
#                 img_msg = find_closest_message(image_messages, scan_timestamp_ns)
                
#                 if img_msg is None:
#                     print(f"[overlay] No matching image found for scan {current_scan_idx}")
#                     current_scan_idx += 1
#                     continue
                
#                 # Process the synchronized frame
#                 if not process_frame(scan_msg, img_msg):
#                     current_scan_idx += 1
#                     continue
                
#                 if not paused:
#                     current_scan_idx += 1
            
#             # Handle key presses
#             key = cv2.waitKey(30 if not paused else 0) & 0xFF
#             if key in (ord('q'), 27):  # ESC to quit
#                 break
#             elif key == ord(' '):  # Space to pause/resume
#                 paused = not paused
#                 print(f'[overlay] {"paused" if paused else "resumed"}')
#             elif key in (ord('n'), 83):  # 'n' or RIGHT arrow to step forward when paused
#                 if paused and current_scan_idx < len(scan_messages) - 1:
#                     current_scan_idx += 1
#                     print(f'[overlay] Step forward to scan {current_scan_idx + 1}')

#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

##### to check if the rosbag is dooing well ################
##python3 - <<'PY'
import rclpy
import message_filters
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import Image
import message_filters
from sensor_msgs.msg import Image, LaserScan
import message_filters
from sensor_msgs.msg import Image, LaserScan
import rclpy
import message_filters
from sensor_msgs.msg import Image, LaserScan

# def callback(img_msg, scan_msg):
#     # Compare timestamps for synchronization
#     print(f"Camera timestamp: {img_msg.header.stamp}")
#     print(f"LiDAR timestamp: {scan_msg.header.stamp}")

#     timestamp_diff = abs(img_msg.header.stamp - scan_msg.header.stamp)
#     tolerance = 0.1  # Tolerance in seconds

#     if timestamp_diff < tolerance:
#         print("The data is synchronized!")
#     else:
#         print(f"WARNING: The data is not synchronized! Time difference: {timestamp_diff} seconds")

# def main():
#     rclpy.init()
#     node = rclpy.create_node('sync_check_node')

#     # Define topic names and message types
#     image_topic = '/oakd/rgb/preview/image_raw'
#     scan_topic = '/scan'

#     image_msg_type = Image
#     scan_msg_type = LaserScan

#     # Debugging: print arguments to be passed to Subscriber
#     print(f"Subscribing to topic: {image_topic} with message type {image_msg_type}")
#     print(f"Subscribing to topic: {scan_topic} with message type {scan_msg_type}")

#     try:
#         # Create subscribers for both camera and LiDAR topics
#         image_sub = message_filters.Subscriber(image_topic, image_msg_type)
#         scan_sub = message_filters.Subscriber(scan_topic, scan_msg_type)

#         # Synchronize messages with slop of 0.1 seconds
#         ats = message_filters.ApproximateTimeSynchronizer([image_sub, scan_sub], queue_size=10, slop=0.1)

#         ats.registerCallback(callback)

#         rclpy.spin(node)  # Keep the node running

#     except Exception as e:
#         print(f"Error while creating subscriber: {e}")
#         print(f"Check the arguments passed to message_filters.Subscriber")
#         return

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# Debugging: Manually print arguments before passing them
topic_image = '/oakd/rgb/preview/image_raw'
topic_scan = '/scan'
image_msg_type = Image
scan_msg_type = LaserScan

print(f"Initializing Subscriber for {topic_image} with {image_msg_type}")
print(f"Initializing Subscriber for {topic_scan} with {scan_msg_type}")

image_sub = message_filters.Subscriber(topic_image, image_msg_type)
scan_sub = message_filters.Subscriber(topic_scan, scan_msg_type)
