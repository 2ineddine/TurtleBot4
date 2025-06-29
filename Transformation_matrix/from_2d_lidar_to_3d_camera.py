from typing import List, Tuple
import numpy as np

Pair = Tuple[int, Tuple[float, float, float], Tuple[float, float, float]]

def lidar_polar_to_camera_xyz_pairs(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    T_lidar_to_camera: np.ndarray,
    z_thresh: float = 1e-2
) -> List[Pair]:
    """
    Returns
    -------
    list[ (scan_idx, (x_cam,y_cam,z_cam), (x_lid,y_lid,z_lid)) ]
    """

    # 1) validity
    ranges = np.asarray(ranges, dtype=float)
    valid  = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    if not np.any(valid):
        return []

    idx = np.where(valid)[0]
    r   = ranges[idx]

    # 2) polar → LiDAR Cartesian
    angles = angle_min + idx * angle_increment
    x_l = r * np.cos(angles)
    y_l = r * np.sin(angles)
    pts_lidar = np.column_stack((x_l, y_l, np.zeros_like(x_l)))  # (N,3)

    # 3) LiDAR → Camera
    R = T_lidar_to_camera[:3, :3]
    t = T_lidar_to_camera[:3, 3]
    pts_cam = (R @ pts_lidar.T + t[:, None]).T

    # 4) keep points in front of camera
    in_front = pts_cam[:, 2] > z_thresh
    if not np.any(in_front):
        return []

    idx        = idx[in_front]
    pts_cam    = pts_cam[in_front]
    pts_lidar  = pts_lidar[in_front]

    # 5) build list
    return [
        (int(i), tuple(map(float, xyz_cam)), tuple(map(float, xyz_lid)))
        for i, xyz_cam, xyz_lid in zip(idx, pts_cam, pts_lidar)
    ]