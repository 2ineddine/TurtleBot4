# #!/usr/bin/env python3
# # main.py – play a ROS-bag, overlay LiDAR on the camera image and let
# #           the user pick point bands.  CSV is written when the window
# #           is finally closed.

# from pathlib import Path
# import yaml, numpy as np, cv2, rclpy, matplotlib.pyplot as plt
# from cv_bridge import CvBridge

# # ───── our own modules ────────────────────────────────────────────────
# from data_synchronization      import load_synchronized_lidar_camera_data
# from from_2d_lidar_to_3d_camera import lidar_polar_to_camera_xyz_pairs
# from from_3dTo2d_camera_frame   import project_points_from_pairs
# from interactive_window   import *            # ← THIS version
# from wrote_data_on_csv          import export_selected_points

# # ───── file locations ────────────────────────────────────────────────
# BAG_DIR         = Path("./TestBag")
# INTRINSICS_YAML = Path("./camera_intrinsics.yaml")
# EXTRINSICS_YAML = Path("./camera_extrinsics.yaml")
# OUT_CSV         = Path("selected_points.csv")

# # ──────────────────────────────────────────────────────────────────────
# def load_yaml_matrix(path: Path) -> np.ndarray:
#     with path.open() as f:
#         return np.array(yaml.safe_load(f)).reshape(4, 4)


# def main() -> None:
#     # 1.  Camera intrinsics
#     with INTRINSICS_YAML.open() as f:
#         intr = yaml.safe_load(f)
#     K = np.array(intr["camera_matrix"]["data"], dtype=float).reshape(3, 3)
#     D = np.array(intr["distortion_coefficients"]["data"], dtype=float)

#     # 2.  LiDAR→camera extrinsics
#     with EXTRINSICS_YAML.open() as f:
#         extr = yaml.safe_load(f)
#     T_L2C = np.array(extr["extrinsic_matrix"], dtype=float).reshape(4, 4)

#     # 3.  Initialise ROS – *only once* per process
#     if not rclpy.ok():
#         rclpy.init()

#     bridge = CvBridge()
#     win    = LiDARCameraWindow(K, D)          # create the UI once
#     plt.ion()
#     plt.show(block=False)                     # map the window immediately

#     last_pairs = None                         # keep latest point list for CSV

#     # ── loop over synchronized packets ──────────────────────────────
#     for scan_msg, image_msg, stamp in load_synchronized_lidar_camera_data(BAG_DIR):
#         while win.paused and not win.quit_requested:
#             plt.pause(0.05)          # let key/mouse events be processed
#         if win.quit_requested:
#             break
#         # 3-D points in **camera** frame
#         pairs = lidar_polar_to_camera_xyz_pairs(
#             ranges=np.asarray(scan_msg.ranges),
#             angle_min=scan_msg.angle_min,
#             angle_increment=scan_msg.angle_increment,
#             range_min=scan_msg.range_min,
#             range_max=scan_msg.range_max,
#             T_lidar_to_camera=T_L2C,
#         )
#         if not pairs:
#             continue
#         print(pairs)
#         last_pairs = pairs                           # remember for CSV

#         scan_idx, uv_px = project_points_from_pairs(pairs, K)  # already pixels
#         pts_cam = np.array([xyz for _, xyz in pairs])

#         # ROS → RGB numpy
#         img_bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

#         # feed one frame
#         if not win.push_frame(
#             img_rgb   = img_rgb,
#             scan_idx  = np.asarray(scan_idx, dtype=int),
#             uv        = uv_px.astype(float),
            
#             timestamp = stamp,
#         ):
#             break                                      # user pressed ‘q’

#         plt.pause(0.001)                               # let GUI redraw
#         if win.quit_requested:                                   # safety check
#             break

#     # ── block until window is closed, then dump CSV ─────────────────
#     selections = win.result()                          # modal – waits for close
#     if selections and last_pairs:
#         export_selected_points(selections, last_pairs, OUT_CSV)

#     if rclpy.ok():
#         rclpy.shutdown()
#     print("✓ finished")

# # ──────────────────────────────────────────────────────────────────────
# if __name__ == "__main__":
#     main()
#@@@@@@@@@@@@
#!/usr/bin/env python3
# main.py – play a ROS-bag, overlay LiDAR on the camera image and let
#           the user pick point bands.  CSV is written when the window
#           is finally closed.

# from pathlib import Path
# import yaml
# import numpy as np
# import cv2
# import rclpy
# import matplotlib.pyplot as plt
# from cv_bridge import CvBridge

# # ───── our own modules ────────────────────────────────────────────────
# from data_synchronization       import load_synchronized_lidar_camera_data
# from from_2d_lidar_to_3d_camera import lidar_polar_to_camera_xyz_pairs
# from from_3dTo2d_camera_frame   import project_points_from_pairs
# from interactive_window         import *   # ← current LiDARCameraWindow
# from wrote_data_on_csv          import export_selected_points

# # ───── file locations ────────────────────────────────────────────────
# BAG_DIR         = Path("./TestBag")
# INTRINSICS_YAML = Path("./camera_intrinsics.yaml")
# EXTRINSICS_YAML = Path("./camera_extrinsics.yaml")
# OUT_CSV         = Path("selected_points.csv")

# # ──────────────────────────────────────────────────────────────────────
# def load_yaml_matrix(path: Path) -> np.ndarray:
#     with path.open() as f:
#         return np.array(yaml.safe_load(f)).reshape(4, 4)


# def main() -> None:
#     # 1.  Camera intrinsics -------------------------------------------------
#     with INTRINSICS_YAML.open() as f:
#         intr = yaml.safe_load(f)
#     K = np.array(intr["camera_matrix"]["data"], dtype=float).reshape(3, 3)
#     D = np.array(intr["distortion_coefficients"]["data"], dtype=float)

#     # 2.  LiDAR→camera extrinsics ------------------------------------------
#     with EXTRINSICS_YAML.open() as f:
#         extr = yaml.safe_load(f)
#     T_L2C = np.array(extr["extrinsic_matrix"], dtype=float).reshape(4, 4)

#     # 3.  Initialise ROS – *only once* per process -------------------------
#     if not rclpy.ok():
#         rclpy.init()

#     bridge = CvBridge()
#     win    = LiDARCameraWindow(K, D)          # create the UI once
#     plt.ion()
#     plt.show(block=False)                     # map the window immediately

#     last_pairs = None                         # keep latest point list for CSV

#     # ── NEW: create a manual iterator so we control when next() is called --
#     data_iter = load_synchronized_lidar_camera_data(BAG_DIR)

#     # ── main playback loop -------------------------------------------------
#     while True:

#         # ── pause gate: hold here while the user is paused -----------------
#         while win.paused and not win.quit_requested:
#             plt.pause(0.05)                  # keep GUI responsive
#         if win.quit_requested:
#             break

#         # ── fetch the next set of ROS messages ----------------------------
#         try:
#             scan_msg, image_msg, stamp = next(data_iter)
#         except StopIteration:
#             break                            # bag finished

#         # 3-D points in **camera** frame -----------------------------------
#         pairs = lidar_polar_to_camera_xyz_pairs(
#             ranges=np.asarray(scan_msg.ranges),
#             angle_min=scan_msg.angle_min,
#             angle_increment=scan_msg.angle_increment,
#             range_min=scan_msg.range_min,
#             range_max=scan_msg.range_max,
#             T_lidar_to_camera=T_L2C,
#         )
#         if not pairs:
#             continue
#         last_pairs = pairs                   # remember for CSV

#         scan_idx, uv_px = project_points_from_pairs(pairs, K)  # pixel coords

#         # ROS → RGB numpy ---------------------------------------------------
#         img_bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

#         # feed one frame to the viewer --------------------------------------
#         if not win.push_frame(
#             img_rgb   = img_rgb,
#             scan_idx  = np.asarray(scan_idx, dtype=int),
#             uv        = uv_px.astype(float),
#             timestamp = stamp,
#         ):
#             break                        # user pressed ‘q’

#         plt.pause(0.001)                 # let GUI redraw
#         if win.quit_requested:
#             break

#     # ── block until window is closed, then dump CSV ------------------------
#     selections = win.result()            # modal – waits for close
#     if selections and last_pairs:
#         export_selected_points(selections, last_pairs, OUT_CSV)

#     if rclpy.ok():
#         rclpy.shutdown()
#     print("✓ finished")


# # ──────────────────────────────────────────────────────────────────────
# if __name__ == "__main__":
#     main()
#@@@@@@@@@@@@@@@@@@2
#!/usr/bin/env python3
# main.py – debug edition: prints trace messages so we can see
#           exactly where play/pause flow stalls.

# from pathlib import Path
# import yaml, numpy as np, cv2, rclpy, matplotlib.pyplot as plt
# from cv_bridge import CvBridge

# # ───── our own modules ────────────────────────────────────────────────
# from data_synchronization       import load_synchronized_lidar_camera_data
# from from_2d_lidar_to_3d_camera import lidar_polar_to_camera_xyz_pairs
# from from_3dTo2d_camera_frame   import project_points_from_pairs
# from interactive_window         import *     # LiDARCameraWindow
# from wrote_data_on_csv          import export_selected_points

# # ───── file locations ────────────────────────────────────────────────
# BAG_DIR         = Path("./TestBag")
# INTRINSICS_YAML = Path("./camera_intrinsics.yaml")
# EXTRINSICS_YAML = Path("./camera_extrinsics.yaml")
# OUT_CSV         = Path("selected_points.csv")

# # ──────────────────────────────────────────────────────────────────────
# def load_yaml_matrix(path: Path) -> np.ndarray:
#     with path.open() as f:
#         return np.array(yaml.safe_load(f)).reshape(4, 4)


# def main() -> None:
#     # 1.  Camera intrinsics -------------------------------------------------
#     with INTRINSICS_YAML.open() as f:
#         intr = yaml.safe_load(f)
#     K = np.array(intr["camera_matrix"]["data"], dtype=float).reshape(3, 3)
#     D = np.array(intr["distortion_coefficients"]["data"], dtype=float)

#     # 2.  LiDAR→camera extrinsics ------------------------------------------
#     with EXTRINSICS_YAML.open() as f:
#         extr = yaml.safe_load(f)
#     T_L2C = np.array(extr["extrinsic_matrix"], dtype=float).reshape(4, 4)

#     # 3.  Initialise ROS – *only once* per process -------------------------
#     if not rclpy.ok():
#         rclpy.init()

#     bridge = CvBridge()
#     win    = LiDARCameraWindow(K, D)          # create the UI once

#     # add a trace whenever the user toggles pause/zoom/quit --------------
#     # # qq

#     plt.ion()
#     plt.show(block=False)

#     last_pairs = None

#     # ── manual iterator so we control when next() is called --------------
#     data_iter = load_synchronized_lidar_camera_data(BAG_DIR)

#     frame_counter = 0

#     # ── main playback loop -------------------------------------------------
#     while True:

#         # ── pause gate -----------------------------------------------------
#         if win.paused:
#             print("# DEBUG  entering PAUSE loop")                       # DEBUG
#         while win.paused and not win.quit_requested:
#             plt.pause(0.05)
#         if win.paused is False:
#             print("# DEBUG  leaving  PAUSE loop")                       # DEBUG
#         if win.quit_requested:
#             print("# DEBUG  quit requested; breaking main loop")        # DEBUG
#             break

#         # ── fetch next ROS messages --------------------------------------
#         try:
#             print("# DEBUG  calling next(data_iter)")                   # DEBUG
#             scan_msg, image_msg, stamp = next(data_iter)
#             print("# DEBUG  next() succeeded")                          # DEBUG
#         except StopIteration:
#             print("# DEBUG  StopIteration – bag finished")              # DEBUG
#             break

#         frame_counter += 1
#         print(f"# DEBUG  processing frame {frame_counter}")             # DEBUG

#         # 3-D points -------------------------------------------------------
#         pairs = lidar_polar_to_camera_xyz_pairs(
#             ranges=np.asarray(scan_msg.ranges),
#             angle_min=scan_msg.angle_min,
#             angle_increment=scan_msg.angle_increment,
#             range_min=scan_msg.range_min,
#             range_max=scan_msg.range_max,
#             T_lidar_to_camera=T_L2C,
#         )
#         if not pairs:
#             print("# DEBUG  empty pairs – skipping frame")              # DEBUG
#             continue
#         last_pairs = pairs

#         scan_idx, uv_px = project_points_from_pairs(pairs, K)

#         # ROS → RGB numpy --------------------------------------------------
#         img_bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

#         # feed frame -------------------------------------------------------
#         push_ok = win.push_frame(
#             img_rgb   = img_rgb,
#             scan_idx  = np.asarray(scan_idx, dtype=int),
#             uv        = uv_px.astype(float),
#             timestamp = stamp,
#         )
#         print(f"# DEBUG  push_frame returned {push_ok}")                # DEBUG
#         if not push_ok:
#             break

#         plt.pause(0.001)
#         if win.quit_requested:
#             break

#     # ── after viewer closes ----------------------------------------------
#     selections = win.result()
#     print(selection)
#     if selections and last_pairs:
#         export_selected_points(selections, last_pairs, OUT_CSV)

#     if rclpy.ok():
#         rclpy.shutdown()
#     print("finished")


# # ──────────────────────────────────────────────────────────────────────
# if __name__ == "__main__":
#     main()
#@@@@@@@@@@@@@@@@@@@@@
from pathlib import Path
import yaml, numpy as np, cv2, rclpy, matplotlib.pyplot as plt
from cv_bridge import CvBridge

# ───── our own modules ────────────────────────────────────────────────
from data_synchronization       import load_synchronized_lidar_camera_data
from from_2d_lidar_to_3d_camera import lidar_polar_to_camera_xyz_pairs
from from_3dTo2d_camera_frame   import project_points_from_pairs
from interactive_window         import *     # LiDARCameraWindow
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
    # 1.  Camera intrinsics -------------------------------------------------
    with INTRINSICS_YAML.open() as f:
        intr = yaml.safe_load(f)
    K = np.array(intr["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    D = np.array(intr["distortion_coefficients"]["data"], dtype=float)

    # 2.  LiDAR→camera extrinsics ------------------------------------------
    with EXTRINSICS_YAML.open() as f:
        extr = yaml.safe_load(f)
    T_L2C = np.array(extr["extrinsic_matrix"], dtype=float).reshape(4, 4)

    # 3.  Initialise ROS – *only once* per process -------------------------
    if not rclpy.ok():
        rclpy.init()

    bridge = CvBridge()
    win    = LiDARCameraWindow(K, D)          # create the UI once

    plt.ion()
    plt.show(block=False)

    last_pairs = None

    # ── manual iterator so we control when next() is called --------------
    data_iter = load_synchronized_lidar_camera_data(BAG_DIR)

    frame_counter = 0

    # ── main playback loop -------------------------------------------------
    try:
        while True:
            # ── pause gate -----------------------------------------------------
            if win.paused:
                print("# DEBUG  entering PAUSE loop")
            
            # FIXED: Use win.quit instead of win.quit_requested
            while win.paused and not win.quit:
                plt.pause(0.05)
                # Check for quit during pause
                if win.quit:
                    break
            
            if win.paused is False:
                print("# DEBUG  leaving  PAUSE loop")
            
            # FIXED: Consistent quit checking
            if win.quit:
                print("# DEBUG  quit requested; breaking main loop")
                break

            # ── fetch next ROS messages --------------------------------------
            try:
                print("# DEBUG  calling next(data_iter)")
                scan_msg, image_msg, stamp = next(data_iter)
                print("# DEBUG  next() succeeded")
            except StopIteration:
                print("# DEBUG  StopIteration – bag finished")
                break

            frame_counter += 1
            print(f"# DEBUG  processing frame {frame_counter}")

            # 3-D points -------------------------------------------------------
            triplets = lidar_polar_to_camera_xyz_pairs(
                ranges=np.asarray(scan_msg.ranges),
                angle_min=scan_msg.angle_min,
                angle_increment=scan_msg.angle_increment,
                range_min=scan_msg.range_min,
                range_max=scan_msg.range_max,
                T_lidar_to_camera=T_L2C,

            )
            
            
            index = [i for i, _, _ in triplets]
            pairs = [cam_xyz for _, cam_xyz, _ in triplets]
            lidar2d = [lid_xyz for _, _, lid_xyz in triplets]  # or full 3D
            if not pairs:
                print("# DEBUG  empty pairs – skipping frame")
                continue
            # last_pairs =  [index,pairs]
            last_pairs = list(zip(index, pairs))   
            index_lidar2d = list(zip(index, lidar2d))

            scan_idx, uv_px = project_points_from_pairs(last_pairs, K)

            # ROS → RGB numpy --------------------------------------------------
            img_bgr = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # feed frame -------------------------------------------------------
            push_ok = win.push_frame(
                index_lidar2d = index_lidar2d,
                img_rgb   = img_rgb,
                scan_idx  = np.asarray(scan_idx, dtype=int),
                uv        = uv_px.astype(float),
                timestamp = stamp,
                
            )
            print(f"# DEBUG  push_frame returned {push_ok}")
            if not push_ok:
                print("# DEBUG  push_frame returned False, breaking")
                break

            # Small pause for UI responsiveness
            plt.pause(0.001)
            
            # Final quit check
            if win.quit:
                print("# DEBUG  quit detected, breaking")
                break

    except KeyboardInterrupt:
        print("# DEBUG  KeyboardInterrupt received")
        win.quit = True
    except Exception as e:
        print(f"# DEBUG  Exception in main loop: {e}")
        win.quit = True

    # ── after viewer closes ----------------------------------------------
    print("# DEBUG  Main loop ended, getting results...")
    
    # OPTION 1: Try to get results using the blocking method
    try:
        selections = win.result()
        print(f"# DEBUG  Got {len(selections)} selections via result()")
    except Exception as e:
        print(f"# DEBUG  Error getting results via result(): {e}")
        # OPTION 2: Fallback to direct access
        selections = win.results
        print(f"# DEBUG  Got {len(selections)} selections via direct access")
    
    # FIXED: Correct variable name
    print(f"# DEBUG  Final selections: {len(selections)} items")
    for i, sel in enumerate(selections):
        print(f"  Selection {i}: {len(sel.rows)} points, frame {sel.frame}")
    
    # Export if we have data
    if selections and last_pairs:
        print("# DEBUG  Exporting selected points...")
        #export_selected_points(selections, last_pairs, OUT_CSV)
        print(f"# DEBUG  Exported to {OUT_CSV}")
    else:
        print(f"# DEBUG  No export: selections={len(selections) if selections else 0}, last_pairs={last_pairs is not None}")

    # Cleanup
    try:
        plt.close('all')
    except:
        pass
        
    if rclpy.ok():
        rclpy.shutdown()
    print("# DEBUG  Finished")


# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()