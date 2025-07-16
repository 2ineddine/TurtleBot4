# TurtleBot4 LiDAR-Camera Calibration

A comprehensive toolkit for LiDAR-camera sensor fusion and calibration on TurtleBot4 robots using ROS2.

## Table of Contents
- [Environment Setup](#environment-setup)
- [First Steps with TurtleBot4](#first-steps-with-turtlebot4)
- [ROS Bag Recording](#ros-bag-recording)
- [Project Structure](#project-structure)

## Environment Setup

### ROS2 for Local VM

First, set up ROS2 for both the Raspberry Pi and your VM. For the virtual machine, we use **Oracle VirtualBox**. You can familiarize yourself with the environment through this [VirtualBox tutorial](https://youtu.be/nvdnQX9UkMY?si=sI5WHlclXq0Tst9_).

#### Ubuntu Version Compatibility

| **ROS 2 Version** | **Compatible Ubuntu Version** |
|-------------------|--------------------------------|
| Humble Hawksbill  | Ubuntu 20.04                  |
| Iron Irwini       | Ubuntu 20.04, 22.04           |
| Jazzy Jalisco     | Ubuntu 22.04, 24.04           |

#### Installation Steps

1. **Install ROS2 Humble**: Follow the [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. **Install TurtleBot4 packages**: Use the [TurtleBot 4 Setup](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_desktop.html) guide
3. **Set up ROS2 workspace** for TurtleBot4

After completing these steps, your ROS2 environment is ready to work.

### ROS2 for Robot's Raspberry Pi

#### SD Card Preparation

Retrieve the SD card to flash it:
- **For Ubuntu native**: Follow the instructions in the [Install latest Raspberry Pi image](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) section
- **For other systems**: Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

#### Create3 Base Firmware Update

Update the base `create3` with the new framework system:
1. Download the firmware from [Create3-I.0.0.FastDDS.swu](https://github.com/iRobotEducation/create3_docs/releases/tag/I.0.0)
2. Insert the flashed SD card into the Raspberry Pi and reboot the robot
3. Press both buttons simultaneously until the light turns blue
4. Wait a few seconds, then connect to the Wi-Fi network `Create3xx`
5. Open a web browser and navigate to `192.168.10.1`
6. Go to **Update** in the top left of the page
7. Upload and install the firmware

For a visual guide, watch: [How to Upgrade TurtleBot 4 to ROS 2 Jazzy | Clearpath Robotics](https://www.youtube.com/watch?v=VsmAVYNyYQs)

#### Robot Connection and Setup

1. Reboot the robot and connect to Wi-Fi `Turtlebot4xx` (depends on each robot)
2. Enter the password (found on the robot's nameplate)
3. Open a terminal and type: `ssh robot_name@10.42.0.1`
4. Enter the password (all information is on the robot's nameplate)

#### Internet Connection and Updates

Connect an Ethernet cable to the robot's Raspberry Pi for internet access:

1. **Set system time**: `sudo date -s "2025-07-16 15:07:00"`
2. **Update system**: 
  - `sudo apt update`
  - `sudo apt upgrade`
3. **Install TurtleBot4 workspace** (same as installed on VM)

**Note**: You need to set the date-time each time you connect, as this is a common issue.

#### Environment Synchronization

Synchronize the VM and robot terminals by executing these commands:

**On the VM:**
```bash
source /opt/ros/jazzy/setup.bash && ros2 launch turtlebot4_bringup lite.launch.py
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_STATIC_PEERS="10.42.0.1"
# Make environment variables permanent:
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
```

**Common Problems Encountered**

- Time synchronization issue: You should set up date-time each time - it's among the problems commonly encountered
- Network connectivity: Ensure Ethernet cable is properly connected for internet access
- Environment variables: Make sure all ROS environment variables are correctly exported

## First Steps with TurtleBot4

Once the robot is powered on, execute the command below from the robot's terminal:
```bash
ros2 launch turtlebot4_bringup lite.launch.py
```

Wait a few seconds and check the terminal for any errors. If there are no errors, source the environment using:
```bash
source /etc/turtlebot4/setup.bash
source /opt/ros/humble/setup.bash
```

Do this in both terminals.

Next, undock the robot by pressing the right button (view from behind). The robot should rotate 180 degrees. Now verify if the topics and nodes are available in both terminals using:
```bash
ros2 topic list
ros2 node list
```

Note: some topics and nodes might be missing — for example, camera topics may not be listed in either terminal, and the wheel node may be missing in the VM terminal.

From the VM's terminal, publish a velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

Then, in another terminal:
```bash
ros2 topic echo /cmd_vel
```

If the robot moves, everything is working fine and you can start running your program from the internal workspace. If it doesn't, you may need to execute it from the Raspberry Pi workspace.

To learn more about ROS 2 nodes, see the [official tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).

## ROS Bag Recording

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
  --compression-mode file \
  --compression-format zstd
```

## Project Structure

```
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
└── [other project files]
```

## Module Descriptions

### Camera Calibration

`calibrate_camera_by_opencv.py`: Camera intrinsic parameter calibration using checkerboard patterns with outlier filtering and quality analysis.

### Landmark Segmentation

`estimate_3d_data_color.py`: Color-based landmark detection and 3D data estimation from video streams using HSV color space analysis.

### Matrix Transformation (LiDAR-Camera Pipeline)

Complete LiDAR-camera sensor fusion and calibration pipeline:

- `main.py`: Main orchestration script integrating all modules for interactive LiDAR-camera calibration
- `data_synchronization.py`: ROS2 bag data synchronization between LiDAR and camera sensors with temporal alignment
- `interactive_window.py`: Interactive matplotlib GUI for manual LiDAR-camera point selection with chessboard plane detection

Coordinate Transformation Pipeline:

- `from_2d_lidar_to_3d_camera.py`: 2D LiDAR polar → 3D camera frame coordinates
- `from_3dTo2d_camera_frame.py`: 3D camera → 2D image plane projection

Calibration Methods:

- `estimate_tranformation_matrix.py`: Optimization-based extrinsic calibration using plane equations
- `extrinsic_matrix_from_ros2.py`: Direct extraction from ROS2 TF static transforms

Visualization and Analysis:

- `plot_tranformation.py`: 3D visualization comparing estimated vs ROS2 transformation matrices with frame coordinate systems
- `wrote_data_on_csv.py`: CSV export utilities for calibration data
- `compute_axis_and_rotation_degre.py`: Analysis tools for rotation and translation metrics

## Getting Started

1. Set up your environment following the Environment Setup section
2. Record sensor data using the ROS Bag Recording commands
3. Run camera calibration using `camera_calibration/calibrate_camera_by_opencv.py`
4. Execute the main calibration pipeline with `matrix_transformation_lidar2camera/main.py`
5. Analyze results using the visualization tools

## Requirements

- Ubuntu 20.04/22.04/24.04 (depending on ROS2 version)
- ROS2 Humble/Iron/Jazzy
- TurtleBot4 with LiDAR and camera sensors
- Python packages: OpenCV, NumPy, SciPy, Matplotlib, pandas

## License

[Add your license information here]

This project is licensed under the [Apache License 2.0](LICENSE).

