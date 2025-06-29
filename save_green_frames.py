import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from typing import Generator, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
import os
from datetime import datetime
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
import sys

bridge = CvBridge()

def compute_color_mask(image: np.ndarray, lower_green=(40, 40, 40), upper_green=(90, 255, 255)):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_green), np.array(upper_green))
    return mask

def open_bag_reader(path: Path) -> SequentialReader:
    if path.is_file() and path.suffix == ".db3":
        uri = str(path.parent)
    else:
        uri = str(path)

    if not Path(uri).exists():
        sys.exit(f"[ERROR] Bag path not found: {uri}")

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=uri, storage_id="sqlite3"),
            ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        )
    except RuntimeError as e:
        sys.exit(f"[ERROR] Could not open bag: {e}")
    return reader

def deserialize_message_safe(raw: bytes, msg_type: any) -> any:
    try:
        return deserialize_message(raw, msg_type)
    except Exception as exc:
        print(f"Deserialization failed for {msg_type}: {exc}")
        return None

def process_rosbag_and_record_green_frames(bag_path: Path):
    from std_msgs.msg import Header
    frame_list = []
    recording = False
    video_writer = None
    output_folder = Path("recorded_frames")
    output_folder.mkdir(exist_ok=True)

    image_topic = "/oakd/rgb/preview/image_raw"
    reader = open_bag_reader(bag_path)
    type_map = {
        image_topic: Image
    }

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != image_topic:
            continue

        image_msg = deserialize_message_safe(raw, Image)
        if image_msg is None:
            continue

        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        mask = compute_color_mask(cv_image)
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        display_frame = cv2.addWeighted(cv_image, 0.7, result, 0.3, 0)
        cv2.imshow("Green Segmentation", display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('b'):
            recording = not recording
            print("[INFO] Recording toggled:", "ON" if recording else "OFF")

        elif key == ord('f'):
            if frame_list:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = output_folder / f"green_segment_{timestamp}.avi"
                height, width, _ = frame_list[0].shape
                video_writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*'XVID'), 10, (width, height))
                for f in frame_list:
                    video_writer.write(f)
                video_writer.release()
                print(f"[INFO] Video saved to {output_path}")
                frame_list.clear()

        if recording:
            frame_list.append(display_frame.copy())

    cv2.destroyAllWindows()

if __name__ == "__main__":
    rosbag_path = Path("/media/zineddine/9D1D-BDBE/Turtlebot4_git/TurtleBot4/color_segmetation/color_bag/color_bag_0.db3")
    if not rosbag_path.exists():
        print("[ERROR] Invalid path")
    else:
        process_rosbag_and_record_green_frames(rosbag_path)

