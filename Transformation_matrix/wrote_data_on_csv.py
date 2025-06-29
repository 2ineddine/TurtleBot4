import csv
from pathlib import Path
from typing import List, Tuple, Dict

import csv
from pathlib import Path
from typing import List, Tuple, Dict

# ← import the new dataclass
from interactive_window import Selection


def export_selected_points(
    selections: List[Selection],                          # ← now dataclass
    point_pairs: List[Tuple[int, Tuple[float, float, float]]], # lidar points collected from the beginning 
    csv_path: str | Path = "selected_points.csv",
) -> None:
    """
    Write one row per LiDAR point listed in every Selection.

    CSV columns: scan_idx, x, y, z, a, b, c, d
    """
    csv_path = Path(csv_path)

    # fast look-up   scan_idx  →  (x,y,z)
    xyz_lut: Dict[int, Tuple[float, float, float]] = {
        idx: xyz for idx, xyz in point_pairs
    }

    with csv_path.open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["scan_idx", "x", "y", "z", "a", "b", "c", "d"])

        for sel in selections:
            a, b, c, d = sel.plane
            for idx in sel.rows:
                if idx not in xyz_lut:          # should not happen
                    continue
                x, y, z = xyz_lut[idx]
                wr.writerow([idx, x, y, z, a, b, c, d])

    print(f"[INFO] wrote {csv_path.resolve()}")
