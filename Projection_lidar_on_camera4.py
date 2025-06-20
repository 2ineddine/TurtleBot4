# This code is correct for the projection the lidar point on the camera 
"""
Overlay 2‑D LiDAR scans on RGB frames in a **rosbag2** – Working version for ROS 2 Humble.

* Standalone: no `rosidl_runtime_py` dependency (falls back to `rclpy.serialization`).
* Robust against missing/invalid quaternions, absent ranges, and weird bags.
* Bag path defaults to **./TestBag**, but you can specify any folder or *.db3.
* Topics, frame ids, output video name, etc. are all CLI‑configurable.
* Provides an `--analyze` flag that prints every topic in the bag and exits.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, TopicMetadata
from sensor_msgs.msg import CameraInfo, LaserScan, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, LookupException, ConnectivityException, ExtrapolationException

# ────────────────────────────────────────── Helpers ────────────────────────────

def transform_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
    """Convert ROS TransformStamped → 4×4 homogeneous matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm < 1e-6:
        R = np.eye(3)
        print("he passed by G point")
    else:
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
        ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def project_points(points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Project 3‑D camera‑frame points → 2‑D pixels."""
    in_front = points_cam[:, 2] > 1e-2
    pts = points_cam[in_front]
    if pts.size == 0:
        return np.empty((0, 2))
    uvw = K @ pts.T
    uv = (uvw[:2] / uvw[2]).T
    return uv

# ────────────────────────────────────── Message type map ───────────────────────

def get_message_type_map() -> Dict[str, Any]:
    return {
        "/tf_static": TFMessage,
        "/tf": TFMessage,
        "/scan": LaserScan,
        "/oakd/rgb/preview/image_raw": Image,
        "/oakd/rgb/preview/camera_info": CameraInfo,
    }


def deserialize_message_safe(raw: bytes, msg_type: Any) -> Optional[Any]:
    try:
        return deserialize_message(raw, msg_type)
    except Exception as exc:
        print(f"⚠️  Deserialization failed for {msg_type}: {exc}")
        return None

# ────────────────────────────────────── Bag utilities ─────────────────────────

def open_bag_reader(path: Path) -> SequentialReader:
    if path.is_file() and path.suffix == ".db3":
        uri = str(path.parent)
    else:
        uri = str(path)
    if not Path(uri).exists():
        sys.exit(f"Bag path not found: {uri}")
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    return reader


def analyze_bag_topics(reader: SequentialReader) -> Dict[str, str]:
    print("📄 Topics in bag:")
    topics = {}
    for meta in reader.get_all_topics_and_types():
        topics[meta.name] = meta.type
        print(f"  • {meta.name}  →  {meta.type}")
    return topics

# ─────────────────────────── First pass: static info collection ───────────────

def collect_static_data(reader: SequentialReader, type_map: Dict[str, Any], lidar_frame: str, camera_frame: str, cam_info_topic: str):
    tf_buf = Buffer()
    cam_info = None
    # NOTE: SequentialReader on ROS 2 Humble has no reset()/seek();
    # We call collect_static_data() immediately after opening the bag, so the
    # reader is already at the beginning—no explicit reset needed.
    processed = 0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        processed += 1
        msg = deserialize_message_safe(raw, type_map.get(topic)) if topic in type_map else None
        if msg is None:
            continue
        if topic == "/tf_static":
            for tf in msg.transforms:
                tf_buf.set_transform_static(tf, "bag")
        elif topic == cam_info_topic and cam_info is None:
            cam_info = msg
        if cam_info is not None:
            try:
                tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
                break
            except LookupException:
                pass
        if processed > 2000:
            break
    if cam_info is None:
        sys.exit("❌ CameraInfo topic not found in bag")
    try:
        tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
    except LookupException:
        sys.exit(f"❌ No static TF from {lidar_frame} to {camera_frame}")
    return tf_buf, cam_info

# ────────────────────────────────────────── Main script ────────────────────────

def parse_args():
    ap = argparse.ArgumentParser(description="Overlay 2‑D LiDAR scans on RGB frames (ROS 2 Humble).")
    ap.add_argument("bag", nargs="?", default="TestBag", type=Path, help="Bag directory (default ./TestBag)")
    ap.add_argument("--lidar-frame", default="rplidar_link")
    ap.add_argument("--camera-frame", default="oakd_rgb_camera_optical_frame")
    ap.add_argument("--dt", type=float, default=0.05, help="Max |Δt| between RGB and scan (s)")
    ap.add_argument("--output-video", metavar="overlay.mp4")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--image-topic", default="/oakd/rgb/preview/image_raw")
    ap.add_argument("--camera-info-topic", default="/oakd/rgb/preview/camera_info")
    ap.add_argument("--analyze", action="store_true", help="List topics and exit")
    return ap.parse_args()


def main():
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    print(f"▶️  Reading bag: {bag_path}")

    rclpy.init()
    bridge = CvBridge()

    reader = open_bag_reader(bag_path)
    available = analyze_bag_topics(reader)
    if args.analyze:
        return

    type_map = get_message_type_map()
    type_map[args.scan_topic] = LaserScan
    type_map[args.image_topic] = Image
    type_map[args.camera_info_topic] = CameraInfo

    tf_buffer, cam_info = collect_static_data(reader, type_map, args.lidar_frame, args.camera_frame, args.camera_info_topic)
    K = np.array(cam_info.k).reshape(3, 3)

    # reopen for pass 2
    reader = open_bag_reader(bag_path)
    writer = None
    if args.output_video:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(args.output_video), fourcc, 20.0, (cam_info.width, cam_info.height))
        print(f"💾 Writing video → {args.output_video}")

    pending_scans: List[Tuple[float, LaserScan]] = []
    processed = 0

    try:
        while reader.has_next():
            topic, raw, t_nsec = reader.read_next()
            if topic not in type_map:
                continue
            msg = deserialize_message_safe(raw, type_map[topic])
            if msg is None:
                continue
            t = t_nsec / 1e9
            if topic == args.scan_topic:
                pending_scans.append((t, msg))
                pending_scans = pending_scans[-300:]
                continue
            if topic != args.image_topic or not pending_scans:
                continue
            times = np.array([s[0] for s in pending_scans])
            idx = int(np.argmin(np.abs(times - t)))
            if abs(times[idx] - t) > args.dt:
                continue
            _, scan = pending_scans.pop(idx)

            r = np.asarray(scan.ranges)
            valid = np.isfinite(r) & (scan.range_min < r) & (r < scan.range_max)
            if not np.any(valid):
                continue
            a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
            x_l = r[valid] * np.cos(a[valid])
            y_l = r[valid] * np.sin(a[valid])
            pts_l = np.column_stack((x_l, y_l, np.zeros_like(x_l), np.ones_like(x_l)))

            T = transform_to_matrix(tf_buffer.lookup_transform(args.camera_frame, args.lidar_frame, TimeMsg()))
            pts_c = (T @ pts_l.T).T[:, :3]
            uv = project_points(pts_c, K).astype(int)

            img_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            processed += 1
            if writer is None:
                plt.imshow(img_rgb)
                in_img = (uv[:, 0] >= 0) & (uv[:, 0] < img_rgb.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < img_rgb.shape[0])
                if np.any(in_img):
                    plt.scatter(uv[in_img, 0], uv[in_img, 1], s=8, c="red", alpha=0.7)
                plt.title(f"Frame {processed}")
                plt.axis("off")
                plt.pause(0.001)
                plt.clf()
            else:
                overlay = img_rgb.copy()
                in_bounds = (uv[:, 0] >= 0) & (uv[:, 0] < overlay.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < overlay.shape[0])
                for u, v in uv[in_bounds]:
                    cv2.circle(overlay, (int(u), int(v)), 2, (255, 0, 0), -1)
                writer.write(cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
            if processed % 50 == 0:
                print(f"🖼️  Processed {processed} frames")
    finally:
        if writer:
            writer.release()
            print(f"✅ Video saved with {processed} frames")
        plt.close("all")
        rclpy.shutdown()
        print(f"🏁 Done. Total frames processed: {processed}")


if __name__ == "__main__":
    main()
