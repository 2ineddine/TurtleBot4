import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import json
import time
from rclpy.qos import qos_profile_sensor_data

# Import everything from the camera_ekf module
from .camera_ekf import (
    K, D, T_base_to_camera, OBJ_PTS,  # Constants
    pose_to_T_matrix, wrap_to_pi,      # Utilities  
    project_world_to_camera_frame,     # Projections
    from_3D_to_2d_camera,
    solve_pnp_from_bbox,
    compute_bearing_and_jacobian,      # Core algorithms
    chose_landmark,
    get_uncertainty_box
)


class CameraEKFNode(Node):
    def __init__(self):
        super().__init__('camera_ekf_node')
        
        print("=" * 80)
        print("GREEN: Detected rectangles (camera)")
        print("BLUE: Projected rectangles (prediction)")
        print("RED: Uncertainty regions (noise)")
        print("=" * 80)
        
        # Constants from original script
        self.HSV_LOWER = (40, 40, 40)
        self.HSV_UPPER = (90, 255, 255)
        self.MIN_PIX = 1
        
        # Landmark dictionary - corrected coordinates (negative X)
        self.LANDMARKS = {
       
            1: np.array([-4.330, -1.340, 0.290], dtype=np.float32),
            2: np.array([-6.080, -1.070, 0.270], dtype=np.float32),
            3: np.array([-7.780, -1.300, 0.300], dtype=np.float32),
            4: np.array([-9.050, -1.300, 0.360], dtype=np.float32),
        }
           
        
        # State variables
        self.x_pred = np.zeros(3, dtype=np.float32)
        self.P_pred = np.eye(3, dtype=np.float32) * 0.1
        self.x_pred_received = False
                
        # CV Bridge for image conversion
        self.bridge = CvBridge()
                
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
        



    def process_frame(self, frame):
        """
        Fixed version with proper measurement association logic
        """
        try:
            img_h, img_w = frame.shape[:2]
            
            # Skip if no valid pose
            if np.allclose(self.x_pred, 0) and not self.x_pred_received:
                cv2.imshow("Clean Camera EKF", frame)
                cv2.waitKey(1)
                self.publish_null_zh()
                return

            # STEP 1: GREEN OBJECT DETECTION
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
            mask = cv2.medianBlur(mask, 5)

            # STEP 2: CREATE LABELED MASK FOR choose() FUNCTION
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            labeled = np.zeros_like(mask, dtype=np.int32)
            valid_detections = {}  # Store detection info by blob ID
            
            for idx, cnt in enumerate(cnts, start=1):
                if cv2.contourArea(cnt) < self.MIN_PIX:
                    continue
                    
                x, y, w, h = cv2.boundingRect(cnt)
                
                # GREEN: Draw detected rectangles by camera
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
                
                # Validate with PnP
                ok, rvec, tvec, img_pts = solve_pnp_from_bbox(x, y, w, h)
                if ok:
                    valid_detections[idx] = {
                        "x": x, "y": y, "w": w, "h": h,
                        "center_x": x + w/2,
                        "center_y": y + h/2,
                        "rvec": rvec,  # ADD THIS
                        "tvec": tvec,
                        "img_pts": img_pts
                    }
                    cv2.fillPoly(labeled, [cnt], idx)

            # STEP 3: PROJECTED RECTANGLES (BLUE) & UNCERTAINTY (RED)
            for lid, centre in self.LANDMARKS.items():
                try:
                    # Project landmark center
                    T_WL = pose_to_T_matrix(self.x_pred)  # Use consistent function
                    P_C_arr = project_world_to_camera_frame(centre[None, :], T_WL)
                    
                    if len(P_C_arr) > 0 and P_C_arr[0][2] > 0.1:  # In front of camera
                        P_C = P_C_arr[0]
                        
                        # Project center to image
                        uv_arr = from_3D_to_2d_camera(P_C[None, :], K, img_w, img_h)
                        
                        if len(uv_arr) > 0:
                            u, v = uv_arr[0]
                            
                            # Project both blue and red rectangles
                            blue_corners, red_corners, blue_valid, red_valid = get_uncertainty_box(centre, self.x_pred, self.P_pred,img_w,img_h, k=2.0)
                            
                            # BLUE: Draw projected rectangle (prediction)
                            if blue_corners is not None and blue_valid and len(blue_corners) > 1:
                                blue_corners_int = blue_corners.astype(int)
                                cv2.polylines(frame, [blue_corners_int], True, (255, 0, 0), 1)
                            
                            # RED: Draw uncertainty rectangle
                            if red_corners is not None and red_valid and len(red_corners) > 1:
                                red_corners_int = red_corners.astype(int)
                                cv2.polylines(frame, [red_corners_int], True, (0, 0, 255), 1)
                                
                except Exception as e:
                    pass

            # STEP 4: PROPER MEASUREMENT ASSOCIATION
            try:
                # Use choose() function properly - it returns the best landmark to observe
                best_landmark_id, predicted_bearing, confidence_score = chose_landmark(
                    self.x_pred, self.P_pred, labeled, self.LANDMARKS
                )
                
                if best_landmark_id is not None and confidence_score > 0:
                    # Get the landmark position
                    landmark_position = self.LANDMARKS[best_landmark_id]
                    print(f"the best lindmark index is {best_landmark_id}")
                    
                    # Find which detection blob corresponds to this landmark
                    selected_detection = self.find_detection_for_landmark(
                        best_landmark_id, 
                        landmark_position, 
                        valid_detections, 
                        labeled
                    )
                    
                    if selected_detection is not None:
                        # Calculate measurement from the selected detection
                        cx = selected_detection["center_x"]
                        z_meas = self.calculate_bearing_measurement(selected_detection)                        
                        # Calculate prediction using the same landmark
                        z_pred, H = compute_bearing_and_jacobian(self.x_pred, landmark_position)
                        
                        # Validate measurement
                        if (np.isfinite(z_meas) and np.isfinite(z_pred) and 
                            H is not None and H.size > 0):

                            
                            # Publish valid measurement
                            self.publish_zh(z_meas, z_pred, H, best_landmark_id)
                            
                            # Highlight the matched detection
                            cv2.rectangle(frame, 
                                        (selected_detection["x"], selected_detection["y"]), 
                                        (selected_detection["x"] + selected_detection["w"], 
                                        selected_detection["y"] + selected_detection["h"]), 
                                        (0, 255, 0), 5)
                            
                            # Add debug info
                            cv2.putText(frame, f"L{best_landmark_id}", 
                                    (int(cx), selected_detection["y"] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        else:
                            #print("no landmark has choosen")
                            self.publish_null_zh()
                    else:
                        #print("no landmark detected by camera")
                        self.publish_null_zh()

                else:
                    #print("No landmark has been detected ")
                    self.publish_null_zh()
                    
            except Exception as e:
                self.publish_null_zh()

            # DISPLAY
            cv2.imshow("Camera EKF", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.publish_null_zh()




    def find_detection_for_landmark(self, landmark_id, landmark_position, valid_detections, labeled):
        """
        Find which detection blob corresponds to the chosen landmark
        """
        try:
            # Project landmark to get expected position
            T_WL = pose_to_T_matrix(self.x_pred)
            P_C_arr = project_world_to_camera_frame(landmark_position[None, :], T_WL)
            
            if len(P_C_arr) == 0:
                return None
                
            P_C = P_C_arr[0]
            if P_C[2] <= 0:
                return None
                
            # Get expected pixel position
            uv_arr = from_3D_to_2d_camera(P_C[None, :],K, 250, 250)
            if len(uv_arr) == 0:
                return None
                
            expected_u, expected_v = uv_arr[0]
            
            # Find the detection closest to expected position
            best_detection = None
            min_distance = float('inf')
            
            for blob_id, detection in valid_detections.items():
                # Calculate distance from detection center to expected position
                dx = detection["center_x"] - expected_u
                dy = detection["center_y"] - expected_v
                distance = np.sqrt(dx*dx + dy*dy)
                
                # Also check if this blob is in a reasonable region
                # (Optional: add additional validation here)
                
                if distance < min_distance and distance < 50:  # Max 50 pixel tolerance
                    min_distance = distance
                    best_detection = detection
            
            return best_detection
            
        except Exception as e:
            return None

    def calculate_bearing_measurement(self, selected_detection):
        # Get PnP results
        rvec = selected_detection["rvec"]  # Rotation vector from solvePnP
        tvec = selected_detection["tvec"]  # Translation vector from solvePnP
        
        # tvec gives 3D position of rectangle center in camera frame
        X_cam = tvec[0][0]  # X coordinate in camera frame
        Z_cam = tvec[2][0]  # Z coordinate in camera frame (depth)
        
        # Compute bearing angle
        bearing_rad = math.atan2(X_cam, Z_cam)
        return bearing_rad


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