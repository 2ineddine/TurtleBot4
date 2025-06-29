from typing import Generator, Tuple, List, Dict, Any, Optional
from pathlib import Path
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time as TimeMsg
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from tf2_ros import Buffer, LookupException
from rclpy.serialization import deserialize_message
import sys


def open_bag_reader(path: Path) -> SequentialReader:
    if path.is_file() and path.suffix == ".db3":
        uri = str(path.parent)
    else:
        uri = str(path)
    if not Path(uri).exists():
        sys.exit(f"Bag path not found: {uri}")
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=uri, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )
    return reader


def deserialize_message_safe(raw: bytes, msg_type: Any) -> Optional[Any]:
    try:
        return deserialize_message(raw, msg_type)
    except Exception as exc:
        print(f"Deserialization failed for {msg_type}: {exc}")
        return None


def collect_static_data(
    reader: SequentialReader,
    type_map: Dict[str, Any],
    lidar_frame: str,
    camera_frame: str,
    cam_info_topic: str
) -> Tuple[Buffer, CameraInfo]:
    tf_buf = Buffer()
    cam_info = None
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
        sys.exit("Error: CameraInfo topic not found in bag")
    try:
        tf_buf.lookup_transform(camera_frame, lidar_frame, TimeMsg())
    except LookupException:
        sys.exit(f"Error: No static TF from {lidar_frame} to {camera_frame}")
    return tf_buf, cam_info


def load_synchronized_lidar_camera_data(
    bag_path: Path,
    scan_topic: str = "/scan",
    image_topic: str = "/oakd/rgb/preview/image_raw",
    camera_info_topic: str = "/oakd/rgb/preview/camera_info",
    lidar_frame: str = "rplidar_link",
    camera_frame: str = "oakd_rgb_camera_optical_frame",
    max_dt: float = 0.05
) -> Generator[
    Tuple[LaserScan, Image, float],
    None,
    None
]:
    #rclpy.init()
    bridge = CvBridge()
    reader = open_bag_reader(bag_path)
    type_map = {
        scan_topic: LaserScan,
        image_topic: Image,
        camera_info_topic: CameraInfo,
        "/tf": TFMessage,
        "/tf_static": TFMessage
    }

    # Load TFs and CameraInfo
    _tf_buffer, cam_info = collect_static_data(
        reader, type_map, lidar_frame, camera_frame, camera_info_topic
    )

    # Reopen reader after static data collection
    reader = open_bag_reader(bag_path)
    pending_scans: List[Tuple[float, LaserScan]] = []

    while reader.has_next():
        topic, raw, t_nsec = reader.read_next()
        if topic not in type_map:
            continue

        msg = deserialize_message_safe(raw, type_map[topic])
        if msg is None:
            continue

        stamp_sec = t_nsec / 1e9  # nanoseconds to float seconds

        if topic == scan_topic:
            pending_scans.append((stamp_sec, msg))
            pending_scans = pending_scans[-300:]  # limit memory
            continue

        if topic != image_topic or not pending_scans:
            continue

        # Synchronize image with closest LiDAR scan
        scan_times = np.array([s[0] for s in pending_scans])
        closest_idx = int(np.argmin(np.abs(scan_times - stamp_sec)))
        if abs(scan_times[closest_idx] - stamp_sec) > max_dt:
            continue  # no good match

        scan_msg = pending_scans.pop(closest_idx)[1]
        yield scan_msg, msg, stamp_sec
