#!/usr/bin/env python3
"""
ROS 2 bag inspector

• Lists every topic and its message type
• Counts messages + computes rate & duration
• Shows a sample CameraInfo, Image and LaserScan (if present)
• Estimates sync between /oakd/rgb/preview/image_raw and /scan

Usage (optional argument):
  python3 inspect_rosbag.py            # auto-detect first bag folder here
  python3 inspect_rosbag.py my_bag/    # explicit bag path
"""

import sys, os, pathlib, rclpy, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan, CameraInfo

# ------------------------------------------------------------------ helpers
def auto_find_bag() -> pathlib.Path | None:
    """Return first sub-folder that contains metadata.yaml."""
    here = pathlib.Path(__file__).resolve().parent
    for p in [here] + list(here.iterdir()):
        if (p / 'metadata.yaml').is_file():
            return p
    return None

def fmt_time(ns: int) -> str:
    return f"{ns/1e9:.3f}s"

# ------------------------------------------------------------------ main
def inspect_bag(bag_path: pathlib.Path) -> None:
    print(f"Opening bag: {bag_path}\n")
    rclpy.init()
    try:
        reader = rosbag2_py.SequentialReader()
        storage_opts   = rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id='sqlite3')
        converter_opts = rosbag2_py.ConverterOptions('cdr', 'cdr')
        reader.open(storage_opts, converter_opts)

        # ------------------------------------------------ topic list
        topics = reader.get_all_topics_and_types()
        if not topics:
            print("!! Bag has no topic metadata")
            return

        print("=== Topics ===")
        for t in topics:
            print(f" • {t.name:40s} {t.type}")

        # ------------------------------------------------ stats pass
        counts, times = {}, {}
        while reader.has_next():
            topic, data, ts = reader.read_next()
            counts.setdefault(topic, 0)
            times .setdefault(topic, [])
            counts[topic] += 1
            times [topic].append(ts)

        print("\n=== Message statistics ===")
        for topic, n in counts.items():
            t0, t1 = times[topic][0], times[topic][-1]
            dur = (t1-t0)/1e9
            hz  = n/dur if dur else 0
            print(f" • {topic:40s} {n:6d} msgs  {dur:6.2f}s  {hz:5.1f} Hz")

        # ------------------------------------------------ sample data
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_opts, converter_opts)
        got_cam = got_img = got_scan = False
        print("\n=== Sample messages ===")
        while reader.has_next() and not (got_cam and got_img and got_scan):
            topic, data, ts = reader.read_next()
            if topic == '/oakd/rgb/preview/camera_info' and not got_cam:
                try:
                    info = deserialize_message(data, CameraInfo)
                    print("CameraInfo:")
                    print(f"  size      {info.width}×{info.height}")
                    print(f"  K matrix  {list(info.k)}")
                    got_cam = True
                except Exception as e:
                    print("  (failed CameraInfo)", e)
            elif topic == '/oakd/rgb/preview/image_raw' and not got_img:
                try:
                    img = deserialize_message(data, Image)
                    print("Image:")
                    print(f"  size      {img.width}×{img.height}")
                    print(f"  encoding  {img.encoding}")
                    print(f"  stamp     {fmt_time(ts)}")
                    got_img = True
                except Exception as e:
                    print("  (failed Image)", e)
            elif topic == '/scan' and not got_scan:
                try:
                    scan = deserialize_message(data, LaserScan)
                    valid = sum(scan.range_min <= r <= scan.range_max for r in scan.ranges)
                    print("LaserScan:")
                    print(f"  points    {len(scan.ranges)} (valid {valid})")
                    print(f"  angle     {scan.angle_min:.2f} → {scan.angle_max:.2f} rad")
                    print(f"  stamp     {fmt_time(ts)}")
                    got_scan = True
                except Exception as e:
                    print("  (failed LaserScan)", e)

        # ------------------------------------------------ sync check
        if '/oakd/rgb/preview/image_raw' in times and '/scan' in times:
            img_ts  = times['/oakd/rgb/preview/image_raw'][:50]
            scan_ts = times['/scan']
            diffs = [min(abs(it-s) for s in scan_ts)/1e6 for it in img_ts]
            avg, mx = sum(diffs)/len(diffs), max(diffs)
            print("\n=== Time-sync ===")
            print(f"  avg Δ  {avg:5.1f} ms   max Δ  {mx:5.1f} ms",
                  "✅ good" if avg<100 else "⚠️  poor")
    finally:
        rclpy.shutdown()

# ------------------------------------------------------------------ entry
# ------------------------------------------------------------------ entry
if __name__ == '__main__':
    here = pathlib.Path(__file__).resolve().parent
    bag_dir = here / "TestBag"
    if not (bag_dir / "metadata.yaml").exists():
        sys.exit(f"ERROR: {bag_dir} is not a valid rosbag2 folder (missing metadata.yaml)")
    inspect_bag(bag_dir)

