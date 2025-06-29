from typing import List, Tuple
import numpy as np

def project_points_from_pairs(
    point_pairs: List[Tuple[int, Tuple[float, float, float]]],
    K: np.ndarray
) -> Tuple[List[int], np.ndarray]:
    """
    Project 3D camera-frame points (paired with scan indices)
    to 2D image pixel coordinates.

    Parameters
    ----------
    point_pairs : list of (int, (x, y, z))
        A list where each entry is a scan index and a 3D point.
    K : (3, 3) array
        Camera intrinsic matrix.

    Returns
    -------
    Tuple:
        - list of scan indices (length N)
        - (N, 2) ndarray of projected (u, v) image points
    """
    if not point_pairs:
        return [], np.empty((0, 2))

    indices, points_3d = zip(*point_pairs)
    points_3d = np.array(points_3d)           # shape (N,3)

    uvw = K @ points_3d.T                     # shape (3,N)
    uv = (uvw[:2] / uvw[2]).T                 # shape (N,2)

    return list(indices), uv
