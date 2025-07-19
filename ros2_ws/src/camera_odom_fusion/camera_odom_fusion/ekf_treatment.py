import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math
from rclpy.qos   import qos_profile_sensor_data
import json
import csv
import os

class EKF_estimation(Node):
    def __init__(self):
        super().__init__('ekf_estimation')
        ## this part is for logs ####

        # Add after your other __init__ code
        self.log_dir = os.path.expanduser('~/ekf_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        # Step counters
        self.meas_step = 0
        self.state_step = 0

        # Measurement log: step, z_pred, z_meas, landmark_id
        self.meas_log_path = os.path.join(self.log_dir, 'meas_log.csv')
        self.meas_log_file = open(self.meas_log_path, 'w', newline='')
        self.meas_log_writer = csv.writer(self.meas_log_file)
        self.meas_log_writer.writerow(['step', 'z_pred', 'z_meas', 'landmark_id'])

        # State log: step, x_pred, y_pred, theta_pred, x_updt, y_updt, theta_updt
        self.state_log_path = os.path.join(self.log_dir, 'state_log.csv')
        self.state_log_file = open(self.state_log_path, 'w', newline='')
        self.state_log_writer = csv.writer(self.state_log_file)
        self.state_log_writer.writerow(['step', 'x_pred', 'y_pred', 'theta_pred', 'x_updt', 'y_updt', 'theta_updt'])
        ##
        # Initial robot state: x_t = [xt, yt, thetat]
        self.x_est = np.array([0.0, 0.0, 0.0])

        # Initial covariance
        self.P_est = np.diag([0.5, 0.5, np.deg2rad(10)])**2

        # Process noise (odometry uncertainty)
        # Represents uncertainty in U = [xu, yu, thetau] measurements
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])**2

        # Measurement noise (camera measurements z_k = [bearing])
        self.R = np.array([[np.deg2rad(5)**2]]) 

        # Store z_h data from topic
        self.z_meas = None
        self.z_pred = None
        self.H = None
        self.landmark_id = None

        # Store previous odometry for calculating deltas
        self.prev_odom = None

        # Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/get_odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.x_pred_publisher = self.create_publisher(String, '/get_xpred', 10)
        
        # Subscribers
        self.z_h_subscriber = self.create_subscription(
            String,
            '/get_z_h',
            self.z_h_callback,
            10
        )

        self.get_logger().info('EKF Estimation Node initialized')

    def z_h_callback(self, msg):
        # Process z_h data
        try:
            z_h_data = json.loads(msg.data)
            
            # Handle None values explicitly
            if z_h_data.get("z_meas") is None:
                self.z_meas = None
                self.z_pred = None
                self.H = None
                self.landmark_id = None
                self.get_logger().debug("Received null z_h data")
            else:
                self.z_meas = float(z_h_data.get("z_meas"))
                self.z_pred = float(z_h_data.get("z_pred"))
                self.H = np.array(z_h_data.get("H"), dtype=np.float32)
                self.landmark_id = int(z_h_data.get("landmark_id"))
                
                self.get_logger().info(
                    f"Received z_h: meas={self.z_meas:.3f}, pred={self.z_pred:.3f}, "
                    f"landmark={self.landmark_id}"
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in z_h_callback: {str(e)}')

    def odom_callback(self, msg):
        """
        Process odometry and run EKF
        Extract current pose and calculate U = [xu, yu, thetau] from odometry evolution
        """
        try:
            # Extract current position and orientation
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            
            # Convert quaternion to euler angle (yaw)
            orientation_q = msg.pose.pose.orientation
            current_theta = self.quaternion_to_euler(orientation_q)
            
            # Calculate odometry deltas if we have previous data
            if self.prev_odom is not None:
                # Calculate displacement in global frame
                dx_global = current_x - self.prev_odom[0]
                dy_global = current_y - self.prev_odom[1]
                dtheta = current_theta - self.prev_odom[2]
                
                # Wrap angle difference
                dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
                
                # Transform global displacement to robot frame at previous position
                cos_prev_theta = math.cos(self.prev_odom[2])
                sin_prev_theta = math.sin(self.prev_odom[2])
                
                # U = [xu, yu, thetau] in robot frame
                xu = dx_global * cos_prev_theta + dy_global * sin_prev_theta
                yu = -dx_global * sin_prev_theta + dy_global * cos_prev_theta
                thetau = dtheta
                
                U = np.array([xu, yu, thetau])
                
                # Only run EKF if we have significant motion to avoid numerical issues
                if np.linalg.norm(U) > 1e-6:
                    self.run_ekf_step(U)
                else:
                    # Still publish current prediction for camera node
                    self.publish_x_pred()
            else:
                # First odometry message, just publish initial state
                self.publish_x_pred()
            
            # Store current odometry for next iteration
            self.prev_odom = np.array([current_x, current_y, current_theta])
            
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def quaternion_to_euler(self, q):
        """
        Convert quaternion to euler angle (yaw only)
        """
        # Calculate yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def run_ekf_step(self, U):
        """
        Complete EKF step (predict + update)
        
        Parameters:
            U: odometry command [xu, yu, thetau]
        
        Returns:
            x_est: updated state estimate
            P_est: updated covariance estimate
        """
        # Prediction step
        x_pred, P_pred = self.predict_ekf(self.x_est, self.P_est, U)
        
        # Update step (if measurement is available from z_h topic)
        if self.z_meas is not None and self.z_pred is not None and self.H is not None:
            self.x_est, self.P_est = self.update_ekf(x_pred, P_pred, self.R)
            self.get_logger().info(
                f"EKF Update: x=[{self.x_est[0]:.3f}, {self.x_est[1]:.3f}, {np.degrees(self.x_est[2]):.1f}°]"
            )
            self.meas_log_writer.writerow([
            self.meas_step,
            self.z_pred,
            self.z_meas,
            self.landmark_id
            ])
            self.meas_log_file.flush()
            self.meas_step += 1

        else:
            # No measurement available, use prediction only
            self.x_est = x_pred
            self.P_est = P_pred
            self.get_logger().debug("EKF Prediction only (no measurement)")
        
        self.state_log_writer.writerow([
        self.state_step,
            x_pred[0], x_pred[1], x_pred[2],  # prediction
            self.x_est[0], self.x_est[1], self.x_est[2]  # estimated (updated) state
        ])
        self.state_log_file.flush()
        self.state_step += 1


        return self.x_est, self.P_est

    def predict_ekf(self, x_t, P, U):
        """
        Prediction step using odometry-based motion model
        
        Parameters:
            x_t: previous state [xt, yt, thetat]
            P: previous covariance matrix
            U: odometry command [xu, yu, thetau]
        
        Returns:
            x_pred: predicted state
            P_pred: predicted covariance
        """
        xu, yu, thetau = U 
        thetat = x_t[2]
        
        # Motion model: transform odometry measurements to global frame
        cos_theta = math.cos(thetat)
        sin_theta = math.sin(thetat)
        
        # Predict state using odometry measurements
        x_pred = x_t.copy()
        x_pred[0] += xu * cos_theta - yu * sin_theta
        x_pred[1] += xu * sin_theta + yu * cos_theta
        x_pred[2] += thetau
        x_pred[2] = (x_pred[2] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Jacobian of motion model with respect to state (Fx)
        Fx = np.eye(3)
        Fx[0, 2] = -xu * sin_theta - yu * cos_theta
        Fx[1, 2] = xu * cos_theta - yu * sin_theta
        
        # Jacobian of motion model with respect to control input (Fu)
        Fu = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        
        # Predict covariance
        P_pred = Fx @ P @ Fx.T + Fu @ self.Q @ Fu.T
        
        # Publish x_pred
        self.publish_x_pred_data(x_pred, P_pred)
        
        return x_pred, P_pred

    def publish_x_pred_data(self, x_pred, P_pred):
        """Helper function to publish x_pred data"""
        try:
            x_pred_data = {
                "x_pred": x_pred.tolist(), 
                "P_pred": P_pred.tolist(),
                "timestamp": self.get_clock().now().to_msg()._sec + self.get_clock().now().to_msg()._nanosec * 1e-9
            }
            x_pred_msg = String()
            x_pred_msg.data = json.dumps(x_pred_data)
            self.x_pred_publisher.publish(x_pred_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing x_pred: {str(e)}')

    def publish_x_pred(self):
        """Publish current state estimate"""
        self.publish_x_pred_data(self.x_est, self.P_est)

    def update_ekf(self, x_pred, P_pred, R):
        """
        Update step using camera measurements from z_h topic
        
        Parameters:
            x_pred: predicted state
            P_pred: predicted covariance
            R: measurement noise covariance (1x1 matrix for bearing)
        
        Returns:
            x_upd: updated state
            P_upd: updated covariance
        """
        # Use values from z_h topic
        if self.z_meas is None or self.z_pred is None or self.H is None:
            return x_pred, P_pred
        
        z_k = self.z_meas
        z_pred = self.z_pred
        print(f"z_meas = {z_k},         z_pred = {z_pred}")
        H = self.H
        
        # Ensure H is the correct shape (1x3)
        if H.shape != (1, 3):
            self.get_logger().error(f"H has wrong shape: {H.shape}, expected (1, 3)")
            return x_pred, P_pred
        
        # Innovation
        y = np.array([z_k - z_pred])
        y[0] = (y[0] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Innovation covariance
        S = H @ P_pred @ H.T + R
        
        # Check for numerical issues
        if np.linalg.det(S) < 1e-10:
            self.get_logger().warn("Innovation covariance S is nearly singular")
            return x_pred, P_pred
        
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Update state
        x_upd = x_pred + K @ y
        x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi  # wrap angle
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ H
        P_upd = I_KH @ P_pred @ I_KH.T + K @ R @ K.T
        
        # Log innovation for debugging
        self.get_logger().debug(
            f"Innovation: y={np.degrees(y[0]):.1f}°, "
            f"S={np.sqrt(S[0,0]):.3f}, "
            f"K_norm={np.linalg.norm(K):.3f}"
        )
        
        return x_upd, P_upd

    def get_state_estimate(self):
        """Return current state estimate x_t = [xt, yt, thetat]"""
        return self.x_est.copy()
    
    def get_covariance_estimate(self):
        """Return current covariance estimate"""
        return self.P_est.copy()

    def __del__(self):
        self.meas_log_file.close()
        self.state_log_file.close()



def main(args=None):
    rclpy.init(args=args)
    
    ekf_node = EKF_estimation()
    
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()