#!/usr/bin/env python3
"""
Hardcoded version of extrinsic matrix extractor.
No command-line args needed — just run it.
"""

from pathlib import Path
import yaml
import numpy as np
import sys

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, LookupException
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

# ────── HARD-CODED PARAMETERS ──────
BAG_PATH = Path("./TestBag")  # ← change this
LIDAR_FRAME = "rplidar_link"
CAMERA_FRAME = "oakd_rgb_camera_optical_frame"
SCAN_TOPIC = "/scan"
IMAGE_TOPIC = "/oakd/rgb/preview/image_raw"
CAMERA_INFO_TOPIC = "/oakd/rgb/preview/camera_info"
OUTPUT_YAML = "extrinsics.yaml"


def transform_to_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    n = (q.x**2 + q.y**2 + q.z**2 + q.w**2)**0.5 or 1.0
    qx, qy, qz, qw = q.x/n, q.y/n, q.z/n, q.w/n
    R = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def open_bag_reader(uri: Path) -> SequentialReader:
    if uri.is_file() and uri.suffix == ".db3":
        root = uri.parent
    else:
        root = uri
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(root), storage_id="sqlite3"),
                ConverterOptions("cdr", "cdr"))
    return reader


def extract_extrinsic_matrix(bag_path: Path) -> np.ndarray:
    reader = open_bag_reader(bag_path)
    tf_buf = Buffer()

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in ("/tf_static", "/tf"):
            continue
        msg = deserialize_message(raw, TFMessage)
        for tf in msg.transforms:
            if topic == "/tf_static":
                tf_buf.set_transform_static(tf, "bag")
            else:
                tf_buf.set_transform(tf, "bag")
        try:
            tf = tf_buf.lookup_transform(CAMERA_FRAME, LIDAR_FRAME, TimeMsg())
            return transform_to_matrix(tf)
        except LookupException:
            continue

    sys.exit(f"[ERROR] No transform from {LIDAR_FRAME} to {CAMERA_FRAME} found.")


def save_to_yaml(T: np.ndarray, path: Path):
    data = {
        "bag_path": str(BAG_PATH.resolve()),
        "lidar_frame": LIDAR_FRAME,
        "camera_frame": CAMERA_FRAME,
        "topics": {
            "scan": SCAN_TOPIC,
            "image": IMAGE_TOPIC,
            "camera_info": CAMERA_INFO_TOPIC,
        },
        "T_lidar_to_camera": {
            "rows": 4,
            "cols": 4,
            "data": T.reshape(-1).tolist()
        }
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f)
    print(f"Saved extrinsics to {path}")


def main():
    print("Extracting extrinsic matrix...")
    rclpy.init()
    T = extract_extrinsic_matrix(BAG_PATH)
    rclpy.shutdown()
    print("Matrix:\n", T)
    save_to_yaml(T, OUTPUT_YAML)


if __name__ == "__main__":
    main()
