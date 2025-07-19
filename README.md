# TurtleBot4 Setup and Camera-Odometry Fusion Guide

## Set up the environment

### ROS2 for local VM

First of all we should set up the ROS2 for both Raspberry Pi and also for VM. For the virtual machine I've used `Oracle VirtualBox`. You can get familiar with the environment over this video: [How to use VirtualBox - Tutorial for Beginners by Kevin Stratvert](https://youtu.be/nvdnQX9UkMY?si=sI5WHlclXq0Tst9_).

#### Ubuntu Version Selection

Now we need to install the Ubuntu version (I've been using both Jazzy and Humble). Here's the table of version compatibilities:

| **ROS 2 Version** | **Compatible Ubuntu Version** |
|-------------------|--------------------------------|
| Humble Hawksbill  | Ubuntu 20.04                  |
| Iron Irwini       | Ubuntu 20.04, 22.04           |
| Jazzy Jalisco     | Ubuntu 22.04, 24.04           |

*Table: Supported ROS 2 and Ubuntu combinations for TurtleBot 4*

#### ROS2 Installation

The `ros2 humble` guide installation is available over this link: [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

#### TurtleBot4 Installation

Now we should install the ROS2 for `TurtleBot4` available also over this link: [TurtleBot 4 Setup](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_desktop.html).

#### Workspace Setup

After that we should get a ROS2 workspace for `TurtleBot4`.

After that our ROS2 is ready to work.

### ROS2 for robot's Raspberry Pi

#### SD Card Preparation

Now is the turn for the robot! We retrieve the SD card to flash it. If you have Ubuntu native you could pursue the instruction in the section [Install latest Raspberry Pi image](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Rather than that you must use [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

#### Create3 Base Firmware Update

Great you're doing well, now we should also update the base `create3` with the new framework system. For reaching that you must download it from [Create3-I.0.0.FastDDS.swu](https://github.com/iRobotEducation/create3_docs/releases/tag/I.0.0).

Now we have the SD card flashed and we retrieve it to Raspberry Pi and we reboot the robot. We push the 2 buttons simultaneously until the light turns to blue, we stay some seconds. The wifi `Create3xx` appears, we connect to it and then we search on web browser `192.168.10.1`. We get into the `Update` in the top left of the page, and finally we upload our firmware.

Actually this video explains it very well: [How to Upgrade TurtleBot 4 to ROS 2 Jazzy | Clearpath Robotics](https://www.youtube.com/watch?v=VsmAVYNyYQs).

#### Robot Connection

Now we reboot the robot and connect to Wi-Fi `Turtlebot4xx` (depends on each robot), enter the password, open a new terminal and type `ssh robot_name@10.42.0.1`, then enter the password (all this information is written on the nameplate on the robot).

#### Internet Setup and System Updates

We connect an `Ethernet-internet` cable on the robot Raspberry Pi. We should first set up the time by `sudo date -s "2025-07-04 15:07:00"`. Now the robot will have access to internet (you should each time set up date-time, it's among the problems encountered).

We carry out:
- `sudo date -s "yyyy-mm-dd hh:mm:ss"`
- `sudo apt update`
- `sudo apt upgrade`

And we carry out the same `turtlebot4 workspace` installed previously on the VM.

#### Terminal Synchronization

We synchronize the two terminals by executing these commands:

**On the VM:**
```bash
source /opt/ros/jazzy/setup.bash && ros2 launch turtlebot4_bringup lite.launch.py
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_STATIC_PEERS="10.42.0.1"
```

**Making environment variables permanent:**
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
```

#### Common Problems Encountered

- **Time synchronization issue**: You should set up date-time each time - it's among the problems commonly encountered
- **Network connectivity**: Ensure Ethernet cable is properly connected for internet access
- **Environment variables**: Make sure all ROS environment variables are correctly exported

## First Steps with TurtleBot4

Once the robot is powered on, execute the command `ros2 launch turtlebot4_bringup lite.launch.py` from the robot's terminal. Wait a few seconds and check the terminal for any errors. If there are no errors, source the environment using:

`source /etc/turtlebot4/setup.bash`  
`source /opt/ros/humble/setup.bash`

Do this in both terminals.

Next, undock the robot by pressing the right button (the view is from behind). The robot should rotate 180 degrees. Now verify if the topics and nodes are available in both terminals using:

- `ros2 topic list`
- `ros2 node list`

Note that some topics and nodes might be missing — for example, camera topics may not be listed in either terminal, and the wheel node may be missing in the VM terminal. These are expected irregularities in some setups.

From the VM's terminal, publish a velocity command using:

`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"`

Then, in another terminal, check the output with:

`ros2 topic echo /cmd_vel`

If the robot moves, everything is working fine and you can start running your program from the internal workspace. If it doesn't, you may need to execute it from the Raspberry Pi workspace, which requires installing all necessary packages and dependencies.

To learn more about launching and interacting with nodes, see the official ROS 2 tutorial:  
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html

## How to Launch a Rosbag

To record a rosbag, use the following command:

```bash
ros2 bag record \
  /oakd/rgb/preview/image_raw \
  /oakd/rgb/preview/camera_info \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /imu \
  /joint_states \
  --output ~/bags/hallway_bag_$(date +%Y%m%d_%H%M) \
  --storage sqlite3 \
  --compression-mode file\
  --compression-format zstd
```

## Let's dive in

#### Project Structure

```bash
Turtlebot4_final/
├── camera_calibration/
│   └── calibrate_camera_by_opencv.py
├── landmark_segmentation/
│   └── estimate_3d_data_color.py
├── matrix_transformation_lidar2camera/
│   ├── compute_axis_and_rotation_degre.py
│   ├── data_synchronization.py
│   ├── estimate_tranformation_matrix.py
│   ├── extrinsic_matrix_from_ros2.py
│   ├── from_2d_lidar_to_3d_camera.py
│   ├── from_3dTo2d_camera_frame.py
│   ├── interactive_window.py
│   ├── main.py
│   ├── plot_tranformation.py
│   └── wrote_data_on_csv.py
└── EKF_ws
      └── camera_odom_fusion/
            ├── camera_odom_fusion/
            │   ├── camera_treatement.py    # Camera processing node
            │   ├── ekf_treatment.py        # EKF estimation node
            │   └── camera_ekf.py           # Core algorithms library
            ├── launch/
            │   └── ekf.launch.py           # System launcher
            ├── setup.py                    # Package configuration
            └── package.xml                 # ROS2 package manifest
```

The project contains:
- **Camera calibration**:  
`camera_calibration/calibrate_camera_by_opencv.py` - Camera intrinsic parameter calibration using checkerboard patterns with outlier filtering and quality analysis

- **Landmark segmentation**:  
`landmark_segmentation/estimate_3d_data_color.py` - Color-based landmark detection and 3D data estimation from video streams using HSV color space analysis

- **Matrix transformation**:  
`matrix_transformation_lidar2camera/` - Complete LiDAR-camera sensor fusion and calibration pipeline:
  - `main.py` - Main orchestration script integrating all modules for interactive LiDAR-camera calibration
  - `data_synchronization.py` - ROS2 bag data synchronization between LiDAR and camera sensors with temporal alignment
  - `interactive_window.py` - Interactive matplotlib GUI for manual LiDAR-camera point selection with chessboard plane detection
  - Coordinate transformation pipeline:
    - `from_2d_lidar_to_3d_camera.py` - 2D LiDAR polar → 3D camera frame coordinates
    - `from_3dTo2d_camera_frame.py` - 3D camera → 2D image plane projection
  - Calibration methods:
    - `estimate_tranformation_matrix.py` - Optimization-based extrinsic calibration using plane equations
    - `extrinsic_matrix_from_ros2.py` - Direct extraction from ROS2 TF static transforms
  - `plot_tranformation.py` - 3D visualization comparing estimated vs ROS2 transformation matrices with frame coordinate systems
  - `wrote_data_on_csv.py` - CSV export utilities for calibration data
  - Analysis tools (`compute_axis_and_rotation_degre.py`)

## Camera-Odometry Fusion Package

### Package Overview
**Package Name**: `camera_odom_fusion`  
**Description**: Real-time camera-based EKF localization system for TurtleBot4

### Project Objective
Implement a **landmark-based Extended Kalman Filter (EKF)** system that fuses:
- **Camera measurements** (bearing angles to colored landmarks)
- **Odometry data** (robot motion estimates)
- **Known landmark map** (world coordinates)

For robust robot localization in indoor environments.

### System Architecture

#### Overall System Design
The camera-odometry fusion system follows a modular architecture with three main components communicating via ROS2 topics. The system implements a distributed sensor fusion approach where camera processing and EKF estimation run as separate nodes, enabling scalability and fault isolation.

#### Core Components

1. **Camera Processing Node** (`camera_treatement.py`)

   This node handles all visual perception tasks and produces bearing measurements for the EKF:

   - **Real-time green landmark detection using HSV color filtering**
     - Converts BGR camera images to HSV color space for robust color segmentation
     - Applies HSV range filtering: Lower bound (40, 40, 40), Upper bound (90, 255, 255)
     - Uses median blur filtering (5×5 kernel) for noise reduction
     - Employs contour detection with `cv2.RETR_EXTERNAL` for blob extraction
     - Filters contours by minimum pixel area (configurable threshold)

   - **3D pose estimation via PnP (Perspective-n-Point) algorithm**
     - Extracts bounding rectangles from detected contours
     - Defines 3D object model: 350×250 mm rectangular landmarks
     - Applies `cv2.solvePnP` with iterative refinement method
     - Computes 6-DOF pose: rotation vector (rvec) and translation vector (tvec)
     - Validates solutions based on reprojection error and geometric constraints

   - **Camera-only bearing measurements to detected landmarks**
     - Extracts 3D position in camera frame: (X_cam, Y_cam, Z_cam) from tvec
     - Computes bearing angle: θ_bearing = arctan2(X_cam, Z_cam)
     - No dependency on landmark world coordinates (pure camera measurement)
     - Provides measurement uncertainty estimation based on detection quality

   - **Data association between detections and known landmarks**
     - Projects known landmark positions to expected pixel coordinates
     - Uses `choose()` algorithm for optimal landmark selection
     - Calculates confidence scores based on:
       - Expected vs actual landmark size ratio (C₁ = S_exp/S_prob)
       - Detection pixel density ratio (C₂ = N_pix/S_exp)
       - Combined score: Score = C₁ × C₂
     - Implements nearest-neighbor association with 50-pixel tolerance
     - Handles occlusions and false detections through probabilistic scoring

2. **EKF Estimation Node** (`ekf_treatment.py`)

   This node implements the core localization algorithm using Extended Kalman Filter:

   - **Extended Kalman Filter implementation**
     - State vector: **x** = [x, y, θ]ᵀ (position and orientation in world frame)
     - Process noise covariance: **Q** = diag([0.1², 0.1², (1°)²])
     - Measurement noise covariance: **R** = [(5°)²] (bearing angle uncertainty)
     - Initial covariance: **P₀** = diag([0.5², 0.5², (10°)²])

   - **Odometry-based motion model prediction**
     - Motion model: **x**_{k+1} = f(**x**_k, **u**_k) where:
       - x_{k+1} = x_k + u_x cos(θ_k) - u_y sin(θ_k)
       - y_{k+1} = y_k + u_x sin(θ_k) + u_y cos(θ_k)
       - θ_{k+1} = θ_k + u_θ
     - Control input: **u** = [u_x, u_y, u_θ]ᵀ (robot-frame odometry)
     - Jacobian computation: **F**_x (state) and **F**_u (control)
     - Covariance prediction: **P**_{k+1|k} = **F**_x **P**_{k|k} **F**_x^T + **F**_u **Q** **F**_u^T

   - **Camera measurement integration and state correction**
     - **Predicted bearing computation (z_pred)**:
       - Δx = x_landmark - x_robot
       - Δy = y_landmark - y_robot
       - φ_world = arctan2(Δy, Δx) (world-frame angle)
       - z_pred = wrap_to_pi(φ_world - θ_robot)
     - **Jacobian matrix computation**:
       - **H** = [∂z_pred/∂x, ∂z_pred/∂y, ∂z_pred/∂θ]
       - **H** = [Δy/r², -Δx/r², -1] where r² = Δx² + Δy²
     - Measurement model: z_pred = h(**x**, **m**_j) (predicted bearing to landmark j)
     - Innovation: **y** = z_meas - z_pred with angle wrapping
     - Innovation covariance: **S** = **H** **P** **H**^T + **R**
     - Kalman gain: **K** = **P** **H**^T **S**^{-1}
     - State update: **x**_{k|k} = **x**_{k|k-1} + **K** **y**
     - Joseph form covariance update for numerical stability

   - **Comprehensive logging system for analysis**
     - Measurement log: `meas_log.csv` (step, z_pred, z_meas, landmark_id)
     - State log: `state_log.csv` (step, x_pred, y_pred, theta_pred, x_updt, y_updt, theta_updt)
     - Real-time CSV writing with automatic timestamping
     - Configurable log directory: `~/ekf_logs/`

3. **Camera-EKF Library** (`camera_ekf.py`)

   This library provides shared mathematical and utility functions for both nodes:

   - **Coordinate transformation utilities**
     - `pose_to_T_matrix()`: Converts (x,y,θ) to 4×4 homogeneous transformation
     - `project_world_to_camera_frame()`: World → camera coordinate transformation
     - `from_3D_to_2d_camera()`: 3D camera → 2D image projection with bounds checking
     - Camera intrinsics integration: K matrix and distortion parameters
     - Multi-frame transformations: World → Base → Camera coordinate chains

   - **Landmark selection algorithms**
     - `chose_landmark()`: Probabilistic landmark selection based on:
       - Visibility constraints (camera field of view)
       - Expected landmark size vs uncertainty region
       - Detection confidence and pixel density
       - Geometric dilution of precision (GDOP) considerations
     - Handles multiple landmark scenarios with optimal selection
     - Implements robust scoring against occlusions and lighting variations

   - **Uncertainty visualization**
     - `get_uncertainty_box()`: Projects uncertainty ellipses to image plane
     - Color-coded visualization: Blue (prediction), Red (uncertainty), Green (detection)
     - Real-time overlay on camera feed for debugging
     - `project_rectangle_corners()`: 3D landmark projection for visualization

   - **Mathematical helper functions**
     - `wrap_to_pi()`: Angle normalization to [-π, π]
     - `compute_bearing_and_jacobian()`: Analytical Jacobian computation
     - `jacobian_centre()`: Numerical Jacobian for complex transformations
     - `solve_pnp_from_bbox()`: 4-point PnP wrapper with validation

#### Inter-Node Communication
- **ROS2 Topics**:
  - `/get_z_h`: Camera measurements (JSON: z_meas, z_pred, H, landmark_id)
  - `/get_xpred`: EKF predictions (JSON: x_pred, P_pred, timestamp)
  - `/get_odom`: Republished odometry for synchronization
- **Data synchronization**: Timestamp-based alignment between camera and odometry
- **Quality of Service**: Sensor data profile for real-time performance

### Usage

#### Individual Node Execution
- Camera node: `ros2 run camera_odom_fusion camera`
- EKF node: `ros2 run camera_odom_fusion ekf`

#### Launch File Execution
`ros2 launch camera_odom_fusion ekf.launch.py`

### ROS2 Topics
- **Subscribed**: `/oakd/rgb/preview/image_raw`, `/odom`
- **Published**: `/get_odom`, `/get_z_h`, `/get_xpred`
- **Internal**: Camera measurements, EKF predictions, state estimates

![RQT-graph](rosgraph.png)
*Figure: RQT-graph*

## Precautions & Problems

> **⚠️ Important Notes:**
> - Always verify the robot's coordinates to ensure the projected landmark points are accurate.
> - Some ROS topics (e.g., camera-related topics) remain unavailable or unlaunched.
> - Some nodes randomly fail to launch, and the reason for this is still unknown.
> - This entire project was executed using `rosbag`. For real-time performance, you must establish communication between the two terminals. Otherwise, the Raspberry Pi will not be able to handle all computations — especially once the LiDAR module is added.
