# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# import json
# import time
# from cv_bridge import CvBridge
# from rclpy.qos   import qos_profile_sensor_data

# # Import everything from the camera_ekf module
# from .camera_ekf import *

# class CameraEKFNode(Node):
#     def __init__(self):
#         super().__init__('camera_ekf_node')
        
#         # Constants from original script
#         self.HSV_LOWER = (40, 40, 40)
#         self.HSV_UPPER = (90, 255, 255)
#         self.MIN_PIX = 20
#         self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        
#         # Landmark dictionary from original script
#         self.LANDMARKS = {
#             1: np.array([4430, 1340, 390], dtype=np.float32),
#             2: np.array([6080, -1270, 270], dtype=np.float32),

#             #1: np.array([4330, 1340, 290], dtype=np.float32),
#             #2: np.array([6080, -1070, 270], dtype=np.float32),
#             #3: np.array([7780, 1300, 300], dtype=np.float32),
#             #4: np.array([9050, 1300, 360], dtype=np.float32),
#         }
        
#         # State variables from original script
#         self.x_pred = np.zeros(3, dtype=np.float32)
#         self.P_pred = np.eye(3, dtype=np.float32) * 0.1
        
#         # CV Bridge for image conversion
#         self.bridge = CvBridge()
        
#         # Current odometry
#         self.current_odom = None
        
#         # Subscribers
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/oakd/rgb/preview/image_raw',
#             self.image_callback,
#             qos_profile_sensor_data
#         )
        
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             qos_profile_sensor_data
#         )
        
#         self.x_pred_subscriber = self.create_subscription(
#             String,
#             '/get_xpred',
#             self.x_pred_callback,
#             10
#         )
        
#         # Publishers
#         self.odom_publisher = self.create_publisher(
#             Odometry,
#             '/get_odom',
#             10
#         )
        
#         self.z_h_publisher = self.create_publisher(
#             String,
#             '/get_z_h',
#             10
#         )
        
#         self.get_logger().info('Camera EKF Node initialized')

#     def x_pred_callback(self, msg):
#         """
#         Callback function for x_pred messages
#         """
#         try:
#             x_pred_data = json.loads(msg.data)
#             self.x_pred = np.array(x_pred_data.get("x_pred"), dtype=np.float32)
#             self.P_pred = np.array(x_pred_data.get("P_pred"), dtype=np.float32)
            
#             self.get_logger().info(f'Received x_pred: {self.x_pred}')
#             self.get_logger().info(f'Received P_pred shape: {self.P_pred.shape}')
            
#         except Exception as e:
#             self.get_logger().error(f'Error in x_pred_callback: {str(e)}')

#     def odom_callback(self, msg):
#         """Handle odometry messages"""
#         self.current_odom = msg
        
#         # Update predicted state from odometry
#         position = msg.pose.pose.position
#         orientation = msg.pose.pose.orientation
        
#         # Convert quaternion to euler angle (yaw)
#         yaw = np.arctan2(
#             2 * (orientation.w * orientation.z + orientation.x * orientation.y),
#             1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
#         )

#         # Publish odometry continuously
#         self.odom_publisher.publish(msg)

#     def image_callback(self, msg):
#         """Handle image messages - main processing loop from original script"""
#         try:
#             # Convert ROS image to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
#             # Process frame using original algorithm
#             self.process_frame(frame)
            
#         except Exception as e:
#             self.get_logger().error(f'Error processing image: {str(e)}')

#     def process_frame(self, frame):
#         try:
#             # 1) Image processing
#             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#             mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
#             mask = cv2.medianBlur(mask, 5)
            
#             # 2) Detect rectangles and label contours
#             cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             labeled = np.zeros_like(mask, dtype=np.int32)
#             rects = []
            
#             for idx, cnt in enumerate(cnts, start=1):
#                 if cv2.contourArea(cnt) < self.MIN_PIX:
#                     continue
#                 x, y, w, h = cv2.boundingRect(cnt)
#                 ok, _, tvec, img_pts = solve_pnp_from_bbox(x, y, w, h)
#                 if ok:
#                     cv2.drawContours(labeled, [cnt], -1, idx, -1)
#                     rects.append({
#                         "id": idx,
#                         "x":x, "y":y, "w":w, "h":h, 
#                         "tvec":tvec, "img_pts":img_pts
#                     })

#             # 3) Choose best landmark
#             best_id, _, _ = choose(self.x_pred, self.P_pred, labeled, self.LANDMARKS)
            
#             # 4) Project and visualize landmarks
#             img_h, img_w = frame.shape[:2]
#             for lid, centre in self.LANDMARKS.items():
#                 corners, is_visible = project_rectangle_corners(centre, self.x_pred, img_w, img_h)
#                 if not is_visible or corners is None or len(corners) == 0:  # Added safety checks
#                     continue
                    
#                 # Draw predicted rectangle (BLUE)
#                 cv2.polylines(frame, [corners.astype(int)], True, (255,0,0), 2)
                
#                 # Draw uncertainty box (RED)
#                 if box := get_uncertainty_box(centre, self.x_pred, self.P_pred):
#                     x0, y0, w0, h0 = box
#                     cv2.rectangle(frame, (x0,y0), (x0+w0,y0+h0), (0,0,255), 1)

#             # 5) Handle BEST landmark measurement
#             if best_id is not None and best_id in self.LANDMARKS:
#                 centre = self.LANDMARKS[best_id]
#                 corners, is_visible = project_rectangle_corners(centre, self.x_pred, img_w, img_h)
                
#                 if is_visible and corners is not None and len(corners) > 0 and rects:  # Added checks
#                     # Find closest detected rectangle
#                     pred_center = np.mean(corners, axis=0)
#                     best_rect = min(
#                         rects,
#                         key=lambda r: np.sqrt(
#                             (r["x"]+r["w"]/2 - pred_center[0])**2 + 
#                             (r["y"]+r["h"]/2 - pred_center[1])**2
#                         )
#                     )
                    
#                     # Verify match is close enough
#                     dist = np.sqrt(
#                         (best_rect["x"]+best_rect["w"]/2 - pred_center[0])**2 +
#                         (best_rect["y"]+best_rect["h"]/2 - pred_center[1])**2
#                     )
#                     if dist < 20:  # pixels
#                         cx = best_rect["x"] + best_rect["w"]/2
#                         z_meas = np.radians(bearing_from_bbox(cx, img_w))
#                         z_pred, H = compute_bearing_and_jacobian(self.x_pred, centre)
#                         self.publish_zh(z_meas, z_pred, H, best_id)
#                     else:
#                         self.get_logger().debug(f"Best match too far: {dist:.1f}px")
#                         self.publish_null_zh()
#                 else:
#                     self.publish_null_zh()
#             else:
#                 self.publish_null_zh()

#             cv2.imshow("frame", frame)
#             cv2.waitKey(1)

#         except Exception as e:
#             self.get_logger().error(f"Frame processing failed: {str(e)}")
#             self.publish_null_zh()


#     def publish_zh(self, z_meas, z_pred, H, landmark_id):
#         """Publish valid measurement"""
#         data = {
#             "z_meas": float(z_meas),
#             "z_pred": float(z_pred),
#             "H": H.tolist(),
#             "landmark_id": int(landmark_id),
#             "timestamp": self.get_clock().now().nanoseconds / 1e9
#         }
#         self.z_h_publisher.publish(String(data=json.dumps(data)))

#     def publish_null_zh(self):
#         """Publish null measurement"""
#         self.z_h_publisher.publish(String(data=json.dumps({
#             "z_meas": None,
#             "z_pred": None,
#             "H": None,
#             "landmark_id": None,
#             "timestamp": self.get_clock().now().nanoseconds / 1e9
#         })))

# def main(args=None):
#     rclpy.init(args=args)
    
#     node = CameraEKFNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Clean up
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#@@@@@@@@@@@@@@@@@@
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# import json
# import time
# from cv_bridge import CvBridge
# from rclpy.qos import qos_profile_sensor_data

# # Import everything from the camera_ekf module
# from .camera_ekf import *

# class CameraEKFNode(Node):
#     def __init__(self):
#         super().__init__('camera_ekf_node')
        
#         print("=" * 80)
#         print("CAMERA EKF NODE WITH FIXED TRANSFORMS")
#         print("=" * 80)
        
#         # Constants from original script
#         self.HSV_LOWER = (40, 40, 40)
#         self.HSV_UPPER = (90, 255, 255)
#         self.MIN_PIX = 20
#         self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        
#         # Landmark dictionary - coordinates in meters
#         self.LANDMARKS = {
#             1: np.array([4.430, 1.340, 0.390], dtype=np.float32),
#             2: np.array([6.080, -1.270, 0.270], dtype=np.float32),
#         }
        
#         print(f"LANDMARKS: {self.LANDMARKS}")
        
#         # State variables
#         self.x_pred = np.zeros(3, dtype=np.float32)
#         self.P_pred = np.eye(3, dtype=np.float32) * 0.1
#         self.x_pred_received = False
        
#         # Debug flags
#         self.debug_mode = True
#         self.frame_count = 0
        
#         # CV Bridge for image conversion
#         self.bridge = CvBridge()
        
#         # Current odometry
#         self.current_odom = None
        
#         # Print current transforms
#         print(f"ORIGINAL T_base_to_camera:\n{T_base_to_camera}")
        
#         # Override the camera transform for debugging
#         # Let's try a simple identity transform first to see if landmarks appear
#         global T_base_to_camera
#         print("\n🔧 OVERRIDING CAMERA TRANSFORM FOR DEBUGGING...")
        
#         # Simple forward-facing camera (camera looks in +X direction)
#         T_base_to_camera = np.array([
#             [0, 0, 1, 0],      # Camera X = Robot Z (depth)
#             [-1, 0, 0, 0],     # Camera Y = -Robot X (left/right)
#             [0, -1, 0, 0],     # Camera Z = -Robot Y (up/down)
#             [0, 0, 0, 1]
#         ], dtype=np.float32)
        
#         print(f"NEW T_base_to_camera:\n{T_base_to_camera}")
        
#         # Subscribers
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/oakd/rgb/preview/image_raw',
#             self.image_callback,
#             qos_profile_sensor_data
#         )
        
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             qos_profile_sensor_data
#         )
        
#         self.x_pred_subscriber = self.create_subscription(
#             String,
#             '/get_xpred',
#             self.x_pred_callback,
#             10
#         )
        
#         # Publishers
#         self.odom_publisher = self.create_publisher(
#             Odometry,
#             '/get_odom',
#             10
#         )
        
#         self.z_h_publisher = self.create_publisher(
#             String,
#             '/get_z_h',
#             10
#         )
        
#         print("INITIALIZATION COMPLETE")
#         print("=" * 80)
        
#         self.get_logger().info('Camera EKF Node with fixed transforms initialized')

#     def x_pred_callback(self, msg):
#         """Callback function for x_pred messages"""
#         try:
#             x_pred_data = json.loads(msg.data)
#             old_x_pred = self.x_pred.copy()
#             self.x_pred = np.array(x_pred_data.get("x_pred"), dtype=np.float32)
#             self.P_pred = np.array(x_pred_data.get("P_pred"), dtype=np.float32)
#             self.x_pred_received = True
            
#             # Log significant changes
#             if not np.allclose(old_x_pred, self.x_pred, atol=0.1):
#                 print(f"📍 Robot moved: {old_x_pred} -> {self.x_pred}")
            
#         except Exception as e:
#             print(f"❌ ERROR in x_pred_callback: {str(e)}")

#     def odom_callback(self, msg):
#         """Handle odometry messages"""
#         self.current_odom = msg
        
#         # Only use odometry as fallback
#         if not self.x_pred_received:
#             position = msg.pose.pose.position
#             orientation = msg.pose.pose.orientation
            
#             yaw = np.arctan2(
#                 2 * (orientation.w * orientation.z + orientation.x * orientation.y),
#                 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
#             )
            
#             old_x_pred = self.x_pred.copy()
#             self.x_pred = np.array([position.x, position.y, yaw], dtype=np.float32)
            
#             if not np.allclose(old_x_pred, self.x_pred, atol=0.1):
#                 print(f"📍 Robot (odom): {old_x_pred} -> {self.x_pred}")

#         self.odom_publisher.publish(msg)

#     def image_callback(self, msg):
#         """Handle image messages"""
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             self.process_frame(frame)
#         except Exception as e:
#             print(f"❌ Image callback error: {str(e)}")

#     def process_frame(self, frame):
#         try:
#             self.frame_count += 1
#             img_h, img_w = frame.shape[:2]
            
#             # Skip if no valid pose
#             if np.allclose(self.x_pred, 0) and not self.x_pred_received:
#                 cv2.putText(frame, "Waiting for pose...", (10, 30), 
#                            self.FONT, 0.7, (0, 0, 255), 2)
#                 cv2.imshow("Camera EKF Fixed", frame)
#                 cv2.waitKey(1)
#                 return

#             # Debug every 60 frames
#             if self.frame_count % 60 == 0:
#                 print(f"\n🎯 FRAME {self.frame_count}")
#                 print(f"Robot pose: {self.x_pred}")
#                 print(f"Image size: {img_w}x{img_h}")

#             # Image processing
#             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#             mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
#             mask = cv2.medianBlur(mask, 5)
            
#             green_pixels = np.sum(mask > 0)

#             # Detect rectangles
#             cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             rects = []
            
#             for idx, cnt in enumerate(cnts, start=1):
#                 if cv2.contourArea(cnt) < self.MIN_PIX:
#                     continue
#                 x, y, w, h = cv2.boundingRect(cnt)
#                 cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
#                 rects.append({"id": idx, "x": x, "y": y, "w": w, "h": h})

#             # Project landmarks with detailed debugging
#             landmarks_visible = 0
#             for lid, centre in self.LANDMARKS.items():
#                 if self.frame_count % 60 == 0:
#                     distance = np.linalg.norm(centre[:2] - self.x_pred[:2])
#                     print(f"🎯 Landmark {lid}: pos={centre}, distance={distance:.2f}m")

#                 try:
#                     # Manual projection with detailed steps
#                     T_WL = pose_to_TWL(self.x_pred)
                    
#                     # Transform landmark to robot base frame
#                     landmark_homogeneous = np.array([centre[0], centre[1], centre[2], 1])
#                     base_coords = T_WL @ landmark_homogeneous
                    
#                     # Transform to camera frame
#                     camera_coords = T_base_to_camera @ base_coords
                    
#                     if self.frame_count % 60 == 0:
#                         print(f"  Base coords: {base_coords[:3]}")
#                         print(f"  Camera coords: {camera_coords[:3]}")
                    
#                     # Check if in front of camera (positive Z in camera frame)
#                     if camera_coords[2] > 0:
#                         # Project to image
#                         u = (K[0, 0] * camera_coords[0] / camera_coords[2]) + K[0, 2]
#                         v = (K[1, 1] * camera_coords[1] / camera_coords[2]) + K[1, 2]
                        
#                         if self.frame_count % 60 == 0:
#                             print(f"  Pixel coords: ({u:.1f}, {v:.1f})")
                        
#                         # Check if in image bounds
#                         if 0 <= u < img_w and 0 <= v < img_h:
#                             # Draw landmark center
#                             cv2.circle(frame, (int(u), int(v)), 8, (255, 0, 255), 2)
#                             cv2.putText(frame, f"L{lid}", (int(u)+10, int(v)), 
#                                        self.FONT, 0.5, (255, 0, 255), 2)
#                             landmarks_visible += 1
                            
#                             # Try to draw rectangle corners
#                             corners, is_visible = project_rectangle_corners(centre, self.x_pred, img_w, img_h)
#                             if is_visible and corners is not None and len(corners) >= 4:
#                                 corners_int = corners.astype(int)
#                                 cv2.polylines(frame, [corners_int], True, (255, 0, 0), 2)
                                
#                             if self.frame_count % 60 == 0:
#                                 print(f"  ✅ LANDMARK {lid} VISIBLE at ({u:.1f}, {v:.1f})")
#                         else:
#                             if self.frame_count % 60 == 0:
#                                 print(f"  ❌ Outside image bounds: ({u:.1f}, {v:.1f})")
#                     else:
#                         if self.frame_count % 60 == 0:
#                             print(f"  ❌ Behind camera: Z={camera_coords[2]:.2f}")
                            
#                 except Exception as e:
#                     if self.frame_count % 60 == 0:
#                         print(f"  ❌ Projection error: {str(e)}")

#             if self.frame_count % 60 == 0:
#                 print(f"📊 Summary: {landmarks_visible}/{len(self.LANDMARKS)} landmarks visible, {len(rects)} detections, {green_pixels} green pixels")

#             # Display info
#             cv2.putText(frame, f"Pose: [{self.x_pred[0]:.2f}, {self.x_pred[1]:.2f}, {np.degrees(self.x_pred[2]):.1f}°]", 
#                        (10, 25), self.FONT, 0.4, (255, 255, 255), 1)
#             cv2.putText(frame, f"Landmarks: {landmarks_visible}/{len(self.LANDMARKS)}", 
#                        (10, 45), self.FONT, 0.4, (255, 255, 255), 1)
#             cv2.putText(frame, f"Detections: {len(rects)}", 
#                        (10, 65), self.FONT, 0.4, (255, 255, 255), 1)
#             cv2.putText(frame, f"Green pixels: {green_pixels}", 
#                        (10, 85), self.FONT, 0.4, (255, 255, 255), 1)

#             # Show windows
#             cv2.imshow("Camera EKF Fixed", frame)
#             cv2.imshow("Green Mask", mask)
#             cv2.waitKey(1)

#             # Publish null measurement for now
#             self.publish_null_zh()

#         except Exception as e:
#             print(f"❌ CRITICAL ERROR: {str(e)}")
#             import traceback
#             traceback.print_exc()

#     def publish_null_zh(self):
#         """Publish null measurement"""
#         try:
#             data = {
#                 "z_meas": None,
#                 "z_pred": None,
#                 "H": None,
#                 "landmark_id": None,
#                 "timestamp": self.get_clock().now().nanoseconds / 1e9
#             }
#             self.z_h_publisher.publish(String(data=json.dumps(data)))
#         except Exception as e:
#             print(f"❌ Publish error: {str(e)}")

# def main(args=None):
#     rclpy.init(args=args)
    
#     node = CameraEKFNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#@@@@@@@@@@@@@@@@@@
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import time
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

# Import everything from the camera_ekf module
from .camera_ekf import *

class CameraEKFNode(Node):
    def __init__(self):
        super().__init__('camera_ekf_node')
        
        print("=" * 80)
        print("🎯 CLEAN RECTANGLES ONLY")
        print("🟢 GREEN: Detected rectangles (camera)")
        print("🔵 BLUE: Projected rectangles (prediction)")
        print("🔴 RED: Uncertainty regions (noise)")
        print("=" * 80)
        
        # Constants from original script
        self.HSV_LOWER = (40, 40, 40)
        self.HSV_UPPER = (90, 255, 255)
        self.MIN_PIX = 20
        self.FONT = cv2.FONT_HERSHEY_SIMPLEX
        
        # Landmark dictionary - corrected coordinates (negative X)
        self.LANDMARKS = {
            1: np.array([-4.430, 1.340, 0.1], dtype=np.float32),
            2: np.array([-6.080, -1.270, 0.1], dtype=np.float32),
        }
        
        # State variables
        self.x_pred = np.zeros(3, dtype=np.float32)
        self.P_pred = np.eye(3, dtype=np.float32) * 0.1
        self.x_pred_received = False
        
        # Debug flags
        self.debug_mode = False  # Disable debug windows
        self.frame_count = 0
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Current odometry
        self.current_odom = None
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.x_pred_subscriber = self.create_subscription(
            String,
            '/get_xpred',
            self.x_pred_callback,
            10
        )
        
        # Publishers
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/get_odom',
            10
        )
        
        self.z_h_publisher = self.create_publisher(
            String,
            '/get_z_h',
            10
        )
        
        print("✅ CLEAN VISUALIZATION MODE READY")
        print("=" * 80)
        
        self.get_logger().info('Clean Rectangles Only Node initialized')

    def project_rectangle_corners_fixed(self, centre_W, x_pred, img_w, img_h):
        """
        Fixed version of rectangle corner projection.
        Returns corners for drawing rectangles.
        """
        try:
            # Get world coordinates of all 4 rectangle corners
            world_corners = centre_W[None, :] + OBJ_PTS  # (4, 3)
            
            # Transform to robot base frame
            T_WL = pose_to_TWL(x_pred)
            
            # Convert to homogeneous coordinates
            world_corners_homo = np.hstack([world_corners, np.ones((4, 1))])  # (4, 4)
            base_corners = (T_WL @ world_corners_homo.T).T  # (4, 4)
            
            # Transform to camera frame
            camera_corners = (T_base_to_camera @ base_corners.T).T  # (4, 4)
            
            # Check which corners are in front of camera
            valid_corners = camera_corners[:, 2] > 0.01  # At least 1cm in front
            
            if not np.any(valid_corners):
                return None, False
            
            # Project valid corners to image
            valid_camera_corners = camera_corners[valid_corners]
            
            # Project using camera intrinsics
            u = (K[0, 0] * valid_camera_corners[:, 0] / valid_camera_corners[:, 2]) + K[0, 2]
            v = (K[1, 1] * valid_camera_corners[:, 1] / valid_camera_corners[:, 2]) + K[1, 2]
            
            projected_corners = np.stack([u, v], axis=1)  # (N, 2)
            
            # Check which corners are within image bounds
            u_valid = (projected_corners[:, 0] >= 0) & (projected_corners[:, 0] < img_w)
            v_valid = (projected_corners[:, 1] >= 0) & (projected_corners[:, 1] < img_h)
            image_valid = u_valid & v_valid
            
            visible_corners = projected_corners[image_valid]
            
            if len(visible_corners) == 0:
                return None, False
            
            # Return the visible corners and whether we have all 4
            all_visible = len(visible_corners) == 4
            
            return visible_corners, all_visible
            
        except Exception as e:
            return None, False

    def x_pred_callback(self, msg):
        """Callback function for x_pred messages"""
        try:
            x_pred_data = json.loads(msg.data)
            self.x_pred = np.array(x_pred_data.get("x_pred"), dtype=np.float32)
            self.P_pred = np.array(x_pred_data.get("P_pred"), dtype=np.float32)
            self.x_pred_received = True
        except Exception as e:
            pass  # Silent error handling

    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.current_odom = msg
        
        # Only use odometry as fallback
        if not self.x_pred_received:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            yaw = np.arctan2(
                2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            )
            
            self.x_pred = np.array([position.x, position.y, yaw], dtype=np.float32)

        self.odom_publisher.publish(msg)

    def image_callback(self, msg):
        """Handle image messages"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame)
        except Exception as e:
            pass  # Silent error handling

    def process_frame(self, frame):
        try:
            self.frame_count += 1
            img_h, img_w = frame.shape[:2]
            
            # Skip if no valid pose
            if np.allclose(self.x_pred, 0) and not self.x_pred_received:
                cv2.imshow("Clean Camera EKF", frame)
                cv2.waitKey(1)
                self.publish_null_zh()
                return

            # =============================================================
            # STEP 1: GREEN OBJECT DETECTION
            # =============================================================
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
            mask = cv2.medianBlur(mask, 5)

            # =============================================================
            # STEP 2: DETECTED RECTANGLES (GREEN)
            # =============================================================
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            labeled = np.zeros_like(mask, dtype=np.int32)
            rects = []
            
            for idx, cnt in enumerate(cnts, start=1):
                if cv2.contourArea(cnt) < self.MIN_PIX:
                    continue
                    
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 🟢 GREEN: Draw detected rectangles by camera
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
                
                # Validate with PnP
                ok, _, tvec, img_pts = solve_pnp_from_bbox(x, y, w, h)
                if ok:
                    cv2.drawContours(labeled, [cnt], -1, idx, -1)
                    rects.append({
                        "id": idx,
                        "x": x, "y": y, "w": w, "h": h, 
                        "tvec": tvec, "img_pts": img_pts
                    })

            # =============================================================
            # STEP 3: PROJECTED RECTANGLES (BLUE) & UNCERTAINTY (RED)
            # =============================================================
            for lid, centre in self.LANDMARKS.items():
                try:
                    # Project landmark center
                    T_WL = pose_to_TWL(self.x_pred)
                    landmark_homo = np.array([centre[0], centre[1], centre[2], 1])
                    base_coords = T_WL @ landmark_homo
                    camera_coords = T_base_to_camera @ base_coords
                    
                    if camera_coords[2] > 0.1:  # In front of camera
                        # Project center to image
                        u = (K[0, 0] * camera_coords[0] / camera_coords[2]) + K[0, 2]
                        v = (K[1, 1] * camera_coords[1] / camera_coords[2]) + K[1, 2]
                        
                        if 0 <= u < img_w and 0 <= v < img_h:
                            
                            # 🔵 BLUE: Project rectangle corners (prediction)
                            corners, all_visible = self.project_rectangle_corners_fixed(centre, self.x_pred, img_w, img_h)
                            
                            if corners is not None and all_visible and len(corners) == 4:
                                # Draw complete predicted rectangle in BLUE
                                corners_int = corners.astype(int)
                                cv2.polylines(frame, [corners_int], True, (255, 0, 0), 1)
                            
                            # 🔴 RED: Draw uncertainty box (noise region)
                            try:
                                box = get_uncertainty_box(centre, self.x_pred, self.P_pred, k=0.5)
                                if box is not None:
                                    x0, y0, w0, h0 = box
                                    x0 = max(0, min(img_w-1, x0))
                                    y0 = max(0, min(img_h-1, y0))
                                    w0 = min(img_w - x0, max(1, w0))
                                    h0 = min(img_h - y0, max(1, h0))
                                    
                                    cv2.rectangle(frame, (x0, y0), (x0+w0, y0+h0), (0, 0, 255), 1)
                            except:
                                pass  # Silent error handling
                            
                except Exception as e:
                    pass  # Silent error handling

            # =============================================================
            # STEP 4: MEASUREMENT PROCESSING
            # =============================================================
            try:
                best_id, best_bearing, best_score = choose(self.x_pred, self.P_pred, labeled, self.LANDMARKS)
                
                if best_id is not None and len(rects) > 0:
                    centre = self.LANDMARKS[best_id]
                    
                    # Find closest detection to predicted position
                    corners, _ = self.project_rectangle_corners_fixed(centre, self.x_pred, img_w, img_h)
                    if corners is not None and len(corners) > 0:
                        pred_center = np.mean(corners, axis=0)
                        
                        # Find closest detection
                        min_dist = float('inf')
                        best_rect = None
                        
                        for rect in rects:
                            rect_center = np.array([rect["x"] + rect["w"]/2, rect["y"] + rect["h"]/2])
                            dist = np.linalg.norm(rect_center - pred_center)
                            if dist < min_dist:
                                min_dist = dist
                                best_rect = rect
                        
                        # If close enough, make measurement
                        if best_rect and min_dist < 50:  # pixels
                            cx = best_rect["x"] + best_rect["w"]/2
                            z_meas = np.radians(bearing_from_bbox(cx, img_w))
                            z_pred, H = compute_bearing_and_jacobian(self.x_pred, centre)
                            
                            if np.isfinite(z_meas) and np.isfinite(z_pred) and H is not None and H.size > 0:
                                self.publish_zh(z_meas, z_pred, H, best_id)
                                
                                # Highlight the matched detection with thicker GREEN border
                                cv2.rectangle(frame, 
                                            (best_rect["x"], best_rect["y"]), 
                                            (best_rect["x"] + best_rect["w"], best_rect["y"] + best_rect["h"]), 
                                            (0, 255, 0), 5)  # Thicker green for matched detection
                            else:
                                self.publish_null_zh()
                        else:
                            self.publish_null_zh()
                    else:
                        self.publish_null_zh()
                else:
                    self.publish_null_zh()
            except Exception as e:
                self.publish_null_zh()

            # =============================================================
            # CLEAN DISPLAY - NO TEXT OVERLAYS
            # =============================================================
            cv2.imshow("Clean Camera EKF", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.publish_null_zh()

    def publish_zh(self, z_meas, z_pred, H, landmark_id):
        """Publish valid measurement"""
        try:
            if isinstance(H, np.ndarray):
                H_list = H.tolist()
            else:
                H_list = H
                
            data = {
                "z_meas": float(z_meas),
                "z_pred": float(z_pred),
                "H": H_list,
                "landmark_id": int(landmark_id),
                "timestamp": self.get_clock().now().nanoseconds / 1e9
            }
            self.z_h_publisher.publish(String(data=json.dumps(data)))
        except Exception as e:
            self.publish_null_zh()

    def publish_null_zh(self):
        """Publish null measurement"""
        try:
            data = {
                "z_meas": None,
                "z_pred": None,
                "H": None,
                "landmark_id": None,
                "timestamp": self.get_clock().now().nanoseconds / 1e9
            }
            self.z_h_publisher.publish(String(data=json.dumps(data)))
        except Exception as e:
            pass  # Silent error handling

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraEKFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()