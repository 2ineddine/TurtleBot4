import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr
from pathlib import Path

odom_data = []
scan_data = []

bag_path = Path('/media/zineddine/9D1D-BDBE/Turtlebot4/odom_lidar_bag')

with AnyReader([bag_path]) as reader:
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/odom':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            odom_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif connection.topic == '/scan':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            ranges = np.array(msg.ranges)
            angles = angle_min + np.arange(len(ranges)) * angle_increment
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            scan_data.append((x, y))

fig, ax = plt.subplots(figsize=(8, 8))
odom_line, = ax.plot([], [], label='Odometry', color='blue')
scan_points = ax.scatter([], [], s=1, c='red', label='LiDAR')

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.legend()
ax.grid()

def update(i):
    if i < len(odom_data):
        x_vals, y_vals = zip(*odom_data[:i+1])
        odom_line.set_data(x_vals, y_vals)
    if i < len(scan_data):
        scan_x, scan_y = scan_data[i]
        scan_points.set_offsets(np.c_[scan_x, scan_y])
    return odom_line, scan_points

ani = FuncAnimation(fig, update, frames=len(odom_data), interval=100, blit=True)
plt.show()
########################################
