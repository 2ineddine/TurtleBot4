# camera_ekf_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node: ros2 run camera_odom_fusion camera
        Node(
            package='camera_odom_fusion',
            executable='camera',
            output='screen'
        ),
        
        # EKF node: ros2 run camera_odom_fusion ekf  
        Node(
            package='camera_odom_fusion',
            executable='ekf',
            output='screen'
        ),
    ])
