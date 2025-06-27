# # lidar_camera_window_min.py
# from __future__ import annotations
# from typing import List, Tuple, Optional
# import numpy as np, matplotlib.pyplot as plt, cv2

# class _ChessboardPlane:
#     def __init__(self, size=(7, 10), square=0.025):
#         self.size = size
#         self.obj  = (np.mgrid[0:size[0], 0:size[1]]
#                      .T.reshape(-1, 2).astype(np.float32)) * square

#     def fit(self, img_rgb, K, D) -> Optional[Tuple[float, float, float, float]]:
#         g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
#         ok, c = cv2.findChessboardCornersSB(g, self.size, cv2.CALIB_CB_EXHAUSTIVE)
#         if not ok:
#             return None
#         cv2.cornerSubPix(g, c, (11,11), (-1,-1),
#                          (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,1e-3))
#         ok, rvec, tvec = cv2.solvePnP(self.obj, c, K, D)
#         if not ok: return None
#         R,_ = cv2.Rodrigues(rvec)
#         n   = (R @ np.array([0,0,1])).ravel(); n /= np.linalg.norm(n)
#         d   = -float(n @ tvec.ravel())
#         return (*n, d)

# class LiDARCameraWindow:
#     def __init__(self, K, D):
#         self.K, self.D   = K, D
#         self.plane_fit   = _ChessboardPlane()
#         self.fig,(self.axT,self.axI)=plt.subplots(1,2,figsize=(14,8),
#                                                   gridspec_kw={'width_ratios':[1,2]})
#         self.axI.set_aspect('equal')
#         self.fig.canvas.mpl_connect('key_press_event', self._on_key)
#         self.fig.canvas.mpl_connect('button_press_event', self._on_click)

#         # current frame data
#         self.img=None; self.uv=None; self.scan_idx=None
#         self.frame_no=0

#         # selection state
#         self.row_a=None; self.row_b=None; self.band_rows=[]
#         self.sel_step=0
#         self.paused=False; self.zoom=False; self.quit=False

#         # results
#         self.selections:List[Tuple[List[int],Tuple[float,float,float,float]]]=[]

#     # ───────── public ─────────
#     def push_frame(self,img_rgb,scan_idx,uv):
#         if self.quit: return
#         if self.paused: plt.pause(0.05); return
#         self.img, self.scan_idx, self.uv = img_rgb, scan_idx, uv
#         self.frame_no += 1
#         self._draw()

#     def result(self):
#         plt.show()
#         return self.selections

#     # ───────── events ─────────
#     def _on_click(self,e):
#         if e.button!=1 or self.img is None: return
#         if not self.paused or self.zoom or e.inaxes!=self.axI: return
#         if e.xdata is None or e.ydata is None: return
#         d2=((self.uv[:,0]-e.xdata)**2+(self.uv[:,1]-e.ydata)**2)
#         row=int(np.argmin(d2))

#         if self.sel_step==0:
#             self.row_a=row; self.sel_step=1
#         elif self.sel_step==1:
#             self.row_b=row
#             self._compute_band_by_index()
#             self.sel_step=2
#             plane=self.plane_fit.fit(self.img,self.K,self.D) or (0,0,0,0)
#             self.selections.append((self.band_rows.copy(),plane))
#         self._draw()

#     # pick all indices between row_a and row_b (inclusive)
#     def _compute_band_by_index(self):
#         i,j=sorted((self.row_a,self.row_b))
#         self.band_rows=list(range(i,j+1))

#     def _on_key(self,e):
#         if e.key=='p': self.paused=not self.paused; self.zoom=False
#         elif e.key=='z' and self.paused:
#             self.zoom=not self.zoom; tb=getattr(self.fig.canvas,'toolbar',None)
#             if tb: tb.zoom()
#         elif e.key=='r': self.row_a=self.row_b=None; self.band_rows=[]; self.sel_step=0
#         elif e.key=='q': self.quit=True; plt.close(self.fig)
#         elif e.key==' ' and self.paused: self.paused=False
#         self._draw()

#     # ───────── drawing ─────────
#     def _draw(self):
#         self.axI.clear()
#         if self.img is not None: self.axI.imshow(self.img)
#         if self.uv is not None:
#             self.axI.scatter(self.uv[:,0],self.uv[:,1],s=3,c='lime',alpha=0.5)
#         # band
#         if self.band_rows:
#             bpts=self.uv[self.band_rows]
#             self.axI.scatter(bpts[:,0],bpts[:,1],s=15,c='black',marker='.')
#         # anchors
#         for r in (self.row_a,self.row_b):
#             if r is not None:
#                 u,v=self.uv[r]; self.axI.scatter(u,v,s=50,c='red',marker='x')
#         self.axI.axis('off')

#         # status pane
#         self.axT.clear(); self.axT.axis('off')
#         status=[f"frame : {self.frame_no}",
#                 f"paused: {self.paused} zoom:{self.zoom}",
#                 f"sel step: {self.sel_step}",
#                 f"band rows: {self.band_rows}"]
#         self.axT.text(0.01,0.99,"\n".join(status),va='top',ha='left',
#                       family='monospace')
#         self.fig.canvas.draw_idle(); plt.pause(0.001)
# #lidar_camera_window_clean.py

#@@@@@@@@@@@@@@@@@@

from __future__ import annotations
from typing import List, Tuple, Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

import numpy as np
import matplotlib.pyplot as plt
import cv2


# ───────────────────────────────── Chess-board helper ─────────────────────────────────
class ChessboardDetector:
    def __init__(self, size: Tuple[int, int] = (7, 10), square: float = 0.025):
        self.size = size
        obj = np.zeros((size[0] * size[1], 3), np.float32)
        obj[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
        self.obj_pts = obj * square

    def detect_and_fit_plane(
        self, img_rgb: np.ndarray, K: np.ndarray, D: np.ndarray
    ) -> Optional[Tuple[float, float, float, float]]:
        g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

        ok, c = cv2.findChessboardCornersSB(
            g, self.size, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        )
        if not ok:
            ok, c = cv2.findChessboardCorners(
                g, self.size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
            )
        if not ok or c.shape[0] < 4:          # ← guard against <4 corners
            return None

        cv2.cornerSubPix(
            g,
            c,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3),
        )

        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c, K, D.reshape(-1, 1))
        if not ok:
            return None

        R, _ = cv2.Rodrigues(rvec)
        n = (R @ np.array([0, 0, 1])).ravel()
        n /= np.linalg.norm(n)
        d = -float(n @ tvec.ravel())
        return (*n, d)


# ───────────────────────────────── Small containers ─────────────────────────────────
class SelMode(Enum):
    BAND = "band"
    INDIV = "individual"


@dataclass
class Selection:
    rows: List[int]
    plane: Tuple[float, float, float, float]
    frame: int
    stamp: Optional[float] = None
    meta: Dict[str, Any] | None = None


# ───────────────────────────────── Main window class ────────────────────────────────
class LiDARCameraWindow:
    def __init__(
        self,
        K: np.ndarray,
        D: np.ndarray,
        mode: SelMode = SelMode.BAND,
        chess_size: Tuple[int, int] = (7, 10),
        square: float = 0.025,
    ):
        self.K, self.D = K, D
        self.detector = ChessboardDetector(chess_size, square)
        self.mode = mode

        # Figure ------------------------------------------------------------------
        self.fig, (self.ax_txt, self.ax_img) = plt.subplots(
            1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1, 2.5]}
        )
        self.ax_img.set_aspect("equal")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        # Current frame data -------------------------------------------------------
        self.img: np.ndarray | None = None
        self.uv: np.ndarray | None = None
        self.scan_idx: np.ndarray | None = None
        self.frame_no = 0
        self.stamp: float | None = None

        # UI state -----------------------------------------------------------------
        self.paused = False
        self.zoom = False
        self.quit = False

        # Selection state ----------------------------------------------------------
        self.a: int | None = None
        self.b: int | None = None
        self.band: list[int] = []
        self.indiv: list[int] = []

        # Results ------------------------------------------------------------------
        self.results: list[Selection] = []
        self.quit_requested = False   # alias – kept in sync below
        self._redraw()

    # ───────────────────────────── public helpers ─────────────────────────────
    # ──────────────────────────────────────────────────────────────
# Pretty status pane (left-hand side)
# ──────────────────────────────────────────────────────────────
    def _current_step(self) -> int:
        """
        0 = nothing, 1 = first anchor chosen, 2 = band ready /
        any number >=2 means a full selection is present.
        """
        if self.mode is SelMode.INDIV:
            return 2 if self.indiv else 0
        if self.a is None:
            return 0
        if self.b is None:
            return 1
        return 2


    def _update_status_panel(self) -> None:
        """Render a verbose, colour-coded status panel à la the
        original ‘big’ visualiser you liked."""
        step  = self._current_step()
        total = len(self.band) if self.mode is SelMode.BAND else len(self.indiv)

        # ── build text lines -------------------------------------------------
        lines: list[str] = []

        # header
        lines.append("═"*46)
        lines.append("        LiDAR–CAMERA  INTERACTIVE STATUS")
        lines.append("═"*46)

        # controls
        lines += [
            "",
            "[Controls]",
            "  p   → pause / play",
            "  z   → toggle zoom (paused)",
            "  r   → reset current selection",
            "  q   → quit",
            "  ␣   → step one frame (paused)",
        ]

        # system state
        lines += [
            "",
            "[System]",
            f"  frame #          : {self.frame_no}",
            f"  mode             : {'PAUSED' if self.paused else 'PLAYING'}",
            f"  zoom enabled     : {'YES' if self.zoom else 'NO'}",
            f"  selection mode   : {self.mode.value}",
        ]

        # selection progress
        if self.mode is SelMode.BAND:
            labels = [
                "Step 1  pause + zoom",
                "Step 2  pick first anchor",
                "Step 3  pick second anchor",
            ]
            lines += [
                "",
                "[Band selection]",
                f"  progress         : {labels[step]}",
                f"  band size        : {total}",
            ]
        else:   # INDIV
            lines += [
                "",
                "[Individual points]",
                f"  points selected  : {total}",
            ]

        # saved selections
        lines += [
            "",
            "[Log]",
            f"  selections saved : {len(self.results)}",
        ]

        # helper hints
        lines += [
            "",
            "[Hints]",
        ]
        if not self.paused:
            lines.append("  » press 'p' to pause before clicking")
        elif self.zoom:
            lines.append("  » disable zoom ('z') to click points")
        else:
            lines.append("  » click in the image to select")

        # ── render on the left axis -----------------------------------------
        self.ax_txt.clear()
        self.ax_txt.axis("off")
        self.ax_txt.text(
            0.02, 0.98,
            "\n".join(lines),
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
            color="black"
        )

    def push_frame(
        self,
        img_rgb: np.ndarray,
        scan_idx: np.ndarray,
        uv: np.ndarray,
        
        timestamp: float | None = None,
    ) -> bool:
        if self.quit:
            return False
        if self.paused:
            plt.pause(0.03)
            return True

        m = min(len(scan_idx), len(uv))
        self.img = img_rgb
        self.scan_idx = scan_idx[:m]
        self.uv = uv[:m]
        self.frame_no += 1
        self.stamp = timestamp
        self._redraw()
        return True

    def result(self) -> list[Selection]:
        plt.show()
        return self.results

    # ───────────────────────── internal: events ─────────────────────────────
    def _on_key(self, e):
        if e.key == "p":
            self.paused = not self.paused
            if not self.paused:
                self.zoom = False
                tb = getattr(self.fig.canvas, "toolbar", None)
                if tb:
                    tb.home()
        elif e.key == "z" and self.paused:
            self.zoom = not self.zoom
            tb = getattr(self.fig.canvas, "toolbar", None)
            if tb:
                tb.zoom()
        elif e.key == "r":
            self._reset()
        elif e.key == "q":
            self.quit = True
            plt.close(self.fig)
        elif e.key == " " and self.paused:
            self.paused = False
        self._redraw()

    def _on_click(self, e):
        if (
            e.button != 1
            or not self.paused
            or self.zoom
            or e.inaxes != self.ax_img
            or self.uv is None
        ):
            return
        d2 = (self.uv[:, 0] - e.xdata) ** 2 + (self.uv[:, 1] - e.ydata) ** 2
        row = int(np.argmin(d2))

        if self.mode is SelMode.BAND:
            self._band_click(row)
        else:
            self._indiv_click(row)
        self._redraw()

    # ───────────────────────── internal: selection logic ────────────────────
    def _band_click(self, row: int):
        if self.a is None:
            self.a = row
        elif self.b is None:
            self.b = row
            i, j = sorted((self.a, self.b))
            self.band = list(range(i, j + 1))
            self._save()
        else:
            self._reset()
            self.a = row

    def _indiv_click(self, row: int):
        if row in self.indiv:
            self.indiv.remove(row)
        else:
            self.indiv.append(row)

    def _save(self):
        if self.img is None or self.uv is None:
            return
        rows = self.band if self.mode is SelMode.BAND else self.indiv
        if not rows:
            return
        plane = self.detector.detect_and_fit_plane(self.img, self.K, self.D) or (
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.results.append(
            Selection(rows=rows.copy(), plane=plane, frame=self.frame_no, stamp=self.stamp)
        )
        self._reset()

    def _reset(self):
        self.a = self.b = None
        self.band.clear()
        self.indiv.clear()

    # ───────────────────────── internal: selection logic ────────────────────
    # inside LiDARCameraWindow  ────────────────────────────────────────────
    # ────────────────────────────────────────────────────────────────────
# replace the old _band_click with this one
# ────────────────────────────────────────────────────────────────────
    def _band_click(self, row: int) -> None:
        """
        First click → set anchor A • Second click → anchor B + BUILD band
        and paint it immediately.  A third click starts a fresh pair.
        """
        if self.a is None:                 # first anchor
            self.a = row
            self._redraw()                 # show anchor right away
            return

        if self.b is None:                 # second anchor
            self.b = row
            self._build_band_by_scan_index()   # fills self.band
            self._redraw()                     # ← PAINTS band instantly
            # (optional) automatically save here, or wait for the user to
            # press the ‘s’ key –   self._save()  # uncomment if desired
            return

        # third click → start a new pair
        self._reset()
        self.a = row
        self._redraw()


    def _build_band_by_scan_index(self) -> None:
        """Fill self.band using original scan indexes, not row numbers."""
        if self.scan_idx is None or self.a is None or self.b is None:
            return

        idx_a = int(self.scan_idx[self.a])
        idx_b = int(self.scan_idx[self.b])

        if idx_a == idx_b:                         # same echo → only anchors
            self.band = [self.a, self.b]
            return

        # unwrap logic: decide whether path crosses the 0-degree gap
        total_bins = int(self.scan_idx.max()) + 1  # usually 360 or 720 etc.
        if abs(idx_a - idx_b) <= total_bins / 2:
            # simple interval (no wrap-around)
            low, high = sorted((idx_a, idx_b))
            mask = (self.scan_idx >= low) & (self.scan_idx <= high)
        else:
            # shorter path wraps around 0°
            low, high = sorted((idx_a, idx_b))
            mask = (self.scan_idx >= high) | (self.scan_idx <= low)

        self.band = np.where(mask)[0].tolist()



    def _redraw(self) -> None:
        # ── right pane ────────────────────────────────────────────────
        self.ax_img.clear()

        if self.img is not None:
            h,  w  = self.img.shape[:2]

            # 1) draw current RGB frame
            self.ax_img.imshow(self.img, origin="upper", zorder=0)

            # 2) **lock axes to image coordinates**
            self.ax_img.set_xlim(0, w)         # left → right
            self.ax_img.set_ylim(h, 0)         # top  → bottom  (invert y)

            # 3) overlay LiDAR projections
            if self.uv is not None:
                self.ax_img.scatter(
                    self.uv[:, 0], self.uv[:, 1],
                    s=4, c="lime", alpha=0.6, marker="o", zorder=1
                )

            # 4) selection graphics
            if self.band:
                pts = self.uv[self.band]
                self.ax_img.scatter(pts[:, 0], pts[:, 1],
                                    s=20, c="black", marker=".",
                                    zorder=2)
            if self.indiv:
                pts = self.uv[self.indiv]
                self.ax_img.scatter(pts[:, 0], pts[:, 1],
                                    s=40, c="red", marker="x",
                                    zorder=2)

            for r in (self.a, self.b):
                if r is not None and r < len(self.uv):
                    u, v = self.uv[r]
                    self.ax_img.scatter(u, v,
                                        s=60, c="red", marker="x",
                                        linewidths=2, zorder=3)

        self.ax_img.axis("off")
        self.ax_img.set_title("Camera with LiDAR overlay")

        # ── left pane (status text) ───────────────────────────────────
        self.ax_txt.clear()
        self.ax_txt.axis("off")
        status = [
            f"frame : {self.frame_no}",
            f"paused: {self.paused}",
            f"zoom  : {self.zoom}",
            f"mode  : {self.mode.value}",
            f"saved : {len(self.results)} selections",
            "",
            "controls:",
            "  p-play/pause   z-zoom   r-reset   q-quit",
            "  space-step (paused)",
        ]
        self.ax_txt.text(
            0.01, 0.99, "\n".join(status),
            va="top", ha="left", family="monospace"
        )

        # ── refresh ───────────────────────────────────────────────────
        self._update_status_panel()

        # refresh GUI
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

# ─────────────────────────────── tiny test harness ───────────────────────────────
    def set_save_callback(self, fn: Callable[[List[int]], None]) -> None:
        """Replace the current save-callback.
        The function receives the list of *row indices* that form the band."""
        self.save_callback = fn

    # convenience: close figure from outside (e.g. ctrl-c handler)
    def close(self) -> None:
        self.quit_requested = True
        plt.close(self.fig)
#@@@@@@@@@@@@@@@@@@@@@@@@
# """
# interactive_window.py  –  LiDAR × Camera band-selection UI
# ==========================================================

# Keys
# ----
# p : play / pause
# z : toggle zoom   (only when paused)
# r : reset current selection
# q : quit window
# ␣ : step one frame while paused
# """

# from __future__ import annotations
# from dataclasses import dataclass
# from enum        import Enum
# from typing      import List, Tuple, Optional, Callable, Dict, Any

# import numpy             as np
# import matplotlib.pyplot as plt
# import cv2


# # ════════════════════════════════════════════════════════════════════
# # Helper – chess-board plane (optional, can be ignored if no board)
# # ════════════════════════════════════════════════════════════════════
# class ChessboardDetector:
#     def __init__(self,
#                  size  : Tuple[int, int] = (7, 10),
#                  square: float           = 0.025):
#         self.size = size
#         obj = np.zeros((size[0]*size[1], 3), np.float32)
#         obj[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
#         self.obj_pts = obj * square

#     def detect_and_fit_plane(self,
#                              img_rgb: np.ndarray,
#                              K      : np.ndarray,
#                              D      : np.ndarray
#                              ) -> Optional[Tuple[float, float, float, float]]:
#         g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
#         ok, c = cv2.findChessboardCornersSB(
#             g, self.size,
#             cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
#         )
#         if not ok:
#             ok, c = cv2.findChessboardCorners(
#                 g, self.size,
#                 cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
#             )
#         if not ok or c.shape[0] < 4:
#             return None

#         cv2.cornerSubPix(
#             g, c, (11, 11), (-1, -1),
#             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
#         )

#         ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c, K, D.reshape(-1, 1))
#         if not ok:
#             return None

#         R, _ = cv2.Rodrigues(rvec)
#         n = (R @ np.array([0, 0, 1])).ravel()
#         n /= np.linalg.norm(n)
#         d = -float(n @ tvec.ravel())
#         return (*n, d)


# # ════════════════════════════════════════════════════════════════════
# # Containers
# # ════════════════════════════════════════════════════════════════════
# class SelMode(Enum):
#     BAND  = "band"       # two anchors → continuous band
#     INDIV = "individual" # click arbitrary points


# @dataclass
# class Selection:
#     rows : List[int]
#     plane: Tuple[float, float, float, float]
#     frame: int
#     stamp: Optional[float] = None
#     meta : Dict[str, Any] | None = None


# # ════════════════════════════════════════════════════════════════════
# # Main: LiDAR × Camera interactive window
# # ════════════════════════════════════════════════════════════════════
# class LiDARCameraWindow:
#     # ----------------------------------------------------------------
#     def __init__(self,
#                  K         : np.ndarray,
#                  D         : np.ndarray,
#                  mode      : SelMode             = SelMode.BAND,
#                  chess_size: Tuple[int, int]     = (7, 10),
#                  square    : float               = 0.025,
#                  save_cb   : Optional[Callable[[List[int]], None]] = None):
#         self.K, self.D         = K, D
#         self.mode              = mode
#         self.detector          = ChessboardDetector(chess_size, square)
#         self.save_callback     = save_cb
#         self.quit_requested    = False   # public alias
#         self.quit = False   

#         # -- figure ----------------------------------------------------
#         self.fig, (self.ax_txt, self.ax_img) = plt.subplots(
#             1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1, 2.5]}
#         )
#         self.ax_img.set_aspect("equal")
#         self.fig.canvas.mpl_connect("key_press_event",   self._on_key)
#         self.fig.canvas.mpl_connect("button_press_event", self._on_click)

#         # -- current frame state --------------------------------------
#         self.img       : np.ndarray | None = None
#         self.uv        : np.ndarray | None = None       # (N,2) pixels
#         self.scan_idx  : np.ndarray | None = None       # (N,) original scan index
#         self.frame_no                    = 0
#         self.stamp   : float | None      = None

#         # -- UI flags --------------------------------------------------
#         self.paused   = False
#         self.zoom     = False
#         self._step    = False   # space-bar single-step

#         # -- selection state ------------------------------------------
#         self.a = self.b = None          # anchor rows
#         self.band : list[int] = []      # rows in the band
#         self.indiv: list[int] = []      # indiv. rows

#         # -- results ---------------------------------------------------
#         self.results: list[Selection] = []

#         self._redraw()

#     # =================================================================
#     # Public helpers
#     # =================================================================
#     def push_frame(self,
#                img_rgb  : np.ndarray,
#                scan_idx : np.ndarray,
#                uv       : np.ndarray,
#                timestamp: float | None = None) -> bool:

#         if self.quit:
#             return False

#         m = min(len(scan_idx), len(uv))
#         self.img      = img_rgb
#         self.scan_idx = scan_idx[:m]
#         self.uv       = uv[:m]
#         self.frame_no += 1
#         self.stamp    = timestamp

#         self._redraw()
        
#         # Reset step flag after processing
#         self._step = False
#         return True
        

#     def result(self) -> List[Selection]:
#         """Block until window closed, then return all selections"""
#         plt.show()
#         return self.results

#     def set_save_callback(self, fn: Callable[[List[int]], None]) -> None:
#         self.save_callback = fn

#     def close(self) -> None:
#         self.quit_requested = True
#         plt.close(self.fig)

#     # =================================================================
#     # Event handlers
#     # =================================================================
#     def _on_key(self, e):
#         if e.key == "p":
#             self.paused = not self.paused
#             if not self.paused:
#                 self.zoom = False
#         elif e.key == "z" and self.paused:
#             self.zoom = not self.zoom
#             tb = getattr(self.fig.canvas, "toolbar", None)
#             if tb:
#                 tb.zoom() if self.zoom else tb.home()
#         elif e.key == "r":
#             self._reset()
#         elif e.key == "q":
#             self.quit = True 
#             self.quit_requested = True
#             self.close()
#         elif e.key == " " and self.paused:
#             self._step = True
#             self.paused = False
#         elif e.key == "s" and self.mode is SelMode.INDIV:
#             self._save()
#         self._redraw()

#     def _on_click(self, e):
#         if (e.button != 1 or not self.paused or self.zoom or
#             e.inaxes != self.ax_img or self.uv is None):
#             return

#         # nearest visible point
#         d2  = (self.uv[:, 0] - e.xdata)**2 + (self.uv[:, 1] - e.ydata)**2
#         row = int(np.argmin(d2))

#         if self.mode is SelMode.BAND:
#             self._band_click(row)
#         else:
#             self._indiv_click(row)
#         self._redraw()

#     # =================================================================
#     # Selection logic
#     # =================================================================
#     def _band_click(self, row: int) -> None:
#         if self.a is None:            # first anchor
#             self.a = row
#             self._redraw()
#             return

#         if self.b is None:            # second anchor
#             self.b = row
#             self._build_band_by_scan_index()

#             self._redraw()            # ① draw band while it still exists
#             self._save()              # ② write CSV + clears selection
#             return

#         # third click → start fresh
#         self._reset()
#         self.a = row
#         self._redraw()


#     def _indiv_click(self, row: int) -> None:
#         if row in self.indiv:
#             self.indiv.remove(row)
#         else:
#             self.indiv.append(row)

#     # -- build band using scan indices so we respect angular order
#     def _build_band_by_scan_index(self) -> None:
#         if self.scan_idx is None or self.a is None or self.b is None:
#             return
#         idx_a = int(self.scan_idx[self.a])
#         idx_b = int(self.scan_idx[self.b])

#         total = int(self.scan_idx.max()) + 1  # e.g. 360, 720 …
#         if abs(idx_a - idx_b) <= total / 2:
#             low, high = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= low) & (self.scan_idx <= high)
#         else:                                  # wrap-around
#             low, high = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= high) | (self.scan_idx <= low)

#         self.band = np.where(mask)[0].tolist()

#     # =================================================================
#     # Save / reset
#     # =================================================================
#     def _save(self) -> None:
#         rows = self.band if self.mode is SelMode.BAND else self.indiv
#         if not rows or self.img is None:
#             return

#         plane = self.detector.detect_and_fit_plane(self.img, self.K, self.D) \
#                 or (0.0, 0.0, 0.0, 0.0)

#         sel = Selection(rows=rows.copy(), plane=plane,
#                         frame=self.frame_no, stamp=self.stamp)
#         self.results.append(sel)

#         if self.save_callback:
#             self.save_callback(rows)

#         self._reset()

#     def _reset(self) -> None:
#         self.a = self.b = None
#         self.band.clear()
#         self.indiv.clear()

#     # =================================================================
#     # Drawing
#     # =================================================================
#     def _current_step(self) -> int:
#         if self.mode is SelMode.INDIV:
#             return 2 if self.indiv else 0
#         if self.a is None:
#             return 0
#         if self.b is None:
#             return 1
#         return 2

#     def _update_status_panel(self) -> None:
#         step   = self._current_step()
#         total  = len(self.band) if self.mode is SelMode.BAND else len(self.indiv)
#         labels = ["(ready)", "first anchor", "band ready"]

#         lines = [
#             "═"*46,
#             "      L I D A R  ×  C A M E R A   S T A T U S",
#             "═"*46,
#             "",
#             "[Controls]",
#             "  p play/pause   z zoom   r reset   q quit",
#             "  space step-frame (paused)",
#             "",
#             "[System]",
#             f"  frame #        : {self.frame_no}",
#             f"  mode           : {'PAUSED' if self.paused else 'PLAYING'}",
#             f"  zoom           : {'ON' if self.zoom else 'OFF'}",
#             "",
#         ]

#         if self.mode is SelMode.BAND:
#             lines += [
#                 "[Band selection]",
#                 f"  progress       : {labels[step]}",
#                 f"  band size      : {total}",
#                 "",
#             ]
#         else:
#             lines += [
#                 "[Individual]",
#                 f"  points chosen  : {total}",
#                 "",
#             ]

#         lines += [
#             "[Log]",
#             f"  selections     : {len(self.results)}"
#         ]

#         self.ax_txt.clear()
#         self.ax_txt.axis("off")
#         self.ax_txt.text(0.02, 0.98, "\n".join(lines),
#                          va="top", ha="left", family="monospace", fontsize=9)

#     def _redraw(self) -> None:
#         # right pane ----------------------------------------------------
#         self.ax_img.clear()
#         if self.img is not None:
#             h, w = self.img.shape[:2]
#             self.ax_img.imshow(self.img, origin="upper", zorder=0)
#             self.ax_img.set_xlim(0, w)
#             self.ax_img.set_ylim(h, 0)

#             if self.uv is not None:
#                 self.ax_img.scatter(self.uv[:, 0], self.uv[:, 1],
#                                     s=4, c="lime", alpha=0.6,
#                                     marker="o", zorder=1)

#             if self.band:
#                 pts = self.uv[self.band]
#                 self.ax_img.scatter(pts[:, 0], pts[:, 1],
#                                     s=20, c="black", marker=".",
#                                     zorder=2)

#             if self.indiv:
#                 pts = self.uv[self.indiv]
#                 self.ax_img.scatter(pts[:, 0], pts[:, 1],
#                                     s=40, c="red", marker="x",
#                                     zorder=2)

#             for anchor in (self.a, self.b):
#                 if anchor is not None and anchor < len(self.uv):
#                     u, v = self.uv[anchor]
#                     self.ax_img.scatter(u, v,
#                                         s=60, c="red", marker="x",
#                                         linewidths=2, zorder=3)

#         self.ax_img.axis("off")
#         self.ax_img.set_title("Camera with LiDAR overlay")

#         # left pane -----------------------------------------------------
#         self._update_status_panel()

#         self.fig.canvas.draw_idle()
#         plt.pause(0.001)

# # ════════════════════════════════════════════════════════════════════
# # Default CSV writer (only used if you do not provide your own)
# # ════════════════════════════════════════════════════════════════════
# def _default_csv(rows: List[int], path: str = "selected_rows.csv") -> None:
#     import csv, os
#     first = not os.path.exists(path)
#     with open(path, "a", newline="") as f:
#         w = csv.writer(f)
#         if first:
#             w.writerow(["row"])
#         for r in rows:
#             w.writerow([r])
#     print(f"[INFO] wrote {len(rows)} row-ids → {path}")

#@@@@@@@@@@@@@@@@@@@@@@
# """
# interactive_window.py  –  LiDAR × Camera band-selection UI
# ==========================================================

# Keys
# ----
# p : play / pause
# z : toggle zoom   (only when paused)
# r : reset current selection
# q : quit window
# ␣ : step one frame while paused
# """

# from __future__ import annotations
# from dataclasses import dataclass
# from enum        import Enum
# from typing      import List, Tuple, Optional, Callable, Dict, Any

# import numpy             as np
# import matplotlib.pyplot as plt
# import cv2


# # ════════════════════════════════════════════════════════════════════
# # Helper – chess-board plane (optional, can be ignored if no board)
# # ════════════════════════════════════════════════════════════════════
# class ChessboardDetector:
#     def __init__(self,
#                  size  : Tuple[int, int] = (7, 10),
#                  square: float           = 0.025):
#         self.size = size
#         obj = np.zeros((size[0]*size[1], 3), np.float32)
#         obj[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
#         self.obj_pts = obj * square

#     def detect_and_fit_plane(self,
#                              img_rgb: np.ndarray,
#                              K      : np.ndarray,
#                              D      : np.ndarray
#                              ) -> Optional[Tuple[float, float, float, float]]:
#         g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
#         ok, c = cv2.findChessboardCornersSB(
#             g, self.size,
#             cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
#         )
#         if not ok:
#             ok, c = cv2.findChessboardCorners(
#                 g, self.size,
#                 cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
#             )
#         if not ok or c.shape[0] < 4:
#             return None

#         cv2.cornerSubPix(
#             g, c, (11, 11), (-1, -1),
#             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
#         )

#         ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c, K, D.reshape(-1, 1))
#         if not ok:
#             return None

#         R, _ = cv2.Rodrigues(rvec)
#         n = (R @ np.array([0, 0, 1])).ravel()
#         n /= np.linalg.norm(n)
#         d = -float(n @ tvec.ravel())
#         return (*n, d)


# # ════════════════════════════════════════════════════════════════════
# # Containers
# # ════════════════════════════════════════════════════════════════════
# class SelMode(Enum):
#     BAND  = "band"       # two anchors → continuous band
#     INDIV = "individual" # click arbitrary points


# @dataclass
# class Selection:
#     rows : List[int]
#     plane: Tuple[float, float, float, float]
#     frame: int
#     stamp: Optional[float] = None
#     meta : Dict[str, Any] | None = None


# # ════════════════════════════════════════════════════════════════════
# # Main: LiDAR × Camera interactive window
# # ════════════════════════════════════════════════════════════════════
# class LiDARCameraWindow:
#     # ----------------------------------------------------------------
#     def __init__(self,
#                  K         : np.ndarray,
#                  D         : np.ndarray,
#                  mode      : SelMode             = SelMode.BAND,
#                  chess_size: Tuple[int, int]     = (7, 10),
#                  square    : float               = 0.025,
#                  save_cb   : Optional[Callable[[List[int]], None]] = None):
#         self.K, self.D         = K, D
#         self.mode              = mode
#         self.detector          = ChessboardDetector(chess_size, square)
#         self.save_callback     = save_cb
#         self.quit_requested    = False   # public alias
#         self.quit = False   

#         # -- figure ----------------------------------------------------
#         self.fig, (self.ax_txt, self.ax_img) = plt.subplots(
#             1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1, 2.5]}
#         )
#         self.ax_img.set_aspect("equal")
#         self.fig.canvas.mpl_connect("key_press_event",   self._on_key)
#         self.fig.canvas.mpl_connect("button_press_event", self._on_click)

#         # -- current frame state --------------------------------------
#         self.img       : np.ndarray | None = None
#         self.uv        : np.ndarray | None = None       # (N,2) pixels
#         self.scan_idx  : np.ndarray | None = None       # (N,) original scan index
#         self.frame_no                    = 0
#         self.stamp   : float | None      = None

#         # -- UI flags --------------------------------------------------
#         self.paused   = False
#         self.zoom     = False
#         self.step_once = False  # Flag for single step
        
#         # -- selection state ------------------------------------------
#         self.a = self.b = None          # anchor rows
#         self.band : list[int] = []      # rows in the band
#         self.indiv: list[int] = []      # indiv. rows

#         # -- results ---------------------------------------------------
#         self.results: list[Selection] = []

#         self._redraw()

#     # =================================================================
#     # Public helpers
#     # =================================================================
#     def push_frame(self,
#                    img_rgb  : np.ndarray,
#                    scan_idx : np.ndarray,
#                    uv       : np.ndarray,
#                    timestamp: float | None = None) -> bool:

#         if self.quit:
#             return False

#         m = min(len(scan_idx), len(uv))
#         self.img      = img_rgb
#         self.scan_idx = scan_idx[:m]
#         self.uv       = uv[:m]
#         self.frame_no += 1
#         self.stamp    = timestamp

#         self._redraw()
#         return True

#     def should_advance_frame(self) -> bool:
#         """Check if we should advance to the next frame"""
#         if self.step_once:
#             self.step_once = False  # Reset flag after use
#             return True
#         return not self.paused

#     def process_events(self) -> None:
#         """Process matplotlib events - call this regularly"""
#         plt.pause(0.01)

#     def result(self) -> List[Selection]:
#         """Block until window closed, then return all selections"""
#         plt.show()
#         return self.results

#     def set_save_callback(self, fn: Callable[[List[int]], None]) -> None:
#         self.save_callback = fn

#     def close(self) -> None:
#         self.quit_requested = True
#         self.quit = True
#         plt.close(self.fig)

#     # =================================================================
#     # Event handlers
#     # =================================================================
#     def _on_key(self, e):
#         if e.key == "p":
#             self.paused = not self.paused
#             print(f"{'PAUSED' if self.paused else 'PLAYING'}")
#             if not self.paused:
#                 self.zoom = False
#         elif e.key == "z" and self.paused:
#             self.zoom = not self.zoom
#             tb = getattr(self.fig.canvas, "toolbar", None)
#             if tb:
#                 tb.zoom() if self.zoom else tb.home()
#         elif e.key == "r":
#             self._reset()
#         elif e.key == "q":
#             self.quit = True 
#             self.quit_requested = True
#             self.close()
#         elif e.key == " " and self.paused:
#             # Single step: set flag to advance one frame
#             self.step_once = True
#             print("STEP")
#         elif e.key == "s" and self.mode is SelMode.INDIV:
#             self._save()
#         self._redraw()

#     def _on_click(self, e):
#         if (e.button != 1 or not self.paused or self.zoom or
#             e.inaxes != self.ax_img or self.uv is None):
#             return

#         # nearest visible point
#         d2  = (self.uv[:, 0] - e.xdata)**2 + (self.uv[:, 1] - e.ydata)**2
#         row = int(np.argmin(d2))

#         if self.mode is SelMode.BAND:
#             self._band_click(row)
#         else:
#             self._indiv_click(row)
#         self._redraw()

#     # =================================================================
#     # Selection logic
#     # =================================================================
#     def _band_click(self, row: int) -> None:
#         if self.a is None:            # first anchor
#             self.a = row
#             self._redraw()
#             return

#         if self.b is None:            # second anchor
#             self.b = row
#             self._build_band_by_scan_index()

#             self._redraw()            # ① draw band while it still exists
#             self._save()              # ② write CSV + clears selection
#             return

#         # third click → start fresh
#         self._reset()
#         self.a = row
#         self._redraw()

#     def _indiv_click(self, row: int) -> None:
#         if row in self.indiv:
#             self.indiv.remove(row)
#         else:
#             self.indiv.append(row)

#     # -- build band using scan indices so we respect angular order
#     def _build_band_by_scan_index(self) -> None:
#         if self.scan_idx is None or self.a is None or self.b is None:
#             return
#         idx_a = int(self.scan_idx[self.a])
#         idx_b = int(self.scan_idx[self.b])

#         total = int(self.scan_idx.max()) + 1  # e.g. 360, 720 …
#         if abs(idx_a - idx_b) <= total / 2:
#             low, high = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= low) & (self.scan_idx <= high)
#         else:                                  # wrap-around
#             low, high = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= high) | (self.scan_idx <= low)

#         self.band = np.where(mask)[0].tolist()

#     # =================================================================
#     # Save / reset
#     # =================================================================
#     def _save(self) -> None:
#         rows = self.band if self.mode is SelMode.BAND else self.indiv
#         if not rows or self.img is None:
#             return

#         plane = self.detector.detect_and_fit_plane(self.img, self.K, self.D) \
#                 or (0.0, 0.0, 0.0, 0.0)

#         sel = Selection(rows=rows.copy(), plane=plane,
#                         frame=self.frame_no, stamp=self.stamp)
#         self.results.append(sel)

#         if self.save_callback:
#             self.save_callback(rows)

#         self._reset()

#     def _reset(self) -> None:
#         self.a = self.b = None
#         self.band.clear()
#         self.indiv.clear()

#     # =================================================================
#     # Drawing
#     # =================================================================
#     def _current_step(self) -> int:
#         if self.mode is SelMode.INDIV:
#             return 2 if self.indiv else 0
#         if self.a is None:
#             return 0
#         if self.b is None:
#             return 1
#         return 2

#     def _update_status_panel(self) -> None:
#         step   = self._current_step()
#         total  = len(self.band) if self.mode is SelMode.BAND else len(self.indiv)
#         labels = ["(ready)", "first anchor", "band ready"]

#         lines = [
#             "═"*46,
#             "      L I D A R  ×  C A M E R A   S T A T U S",
#             "═"*46,
#             "",
#             "[Controls]",
#             "  p play/pause   z zoom   r reset   q quit",
#             "  space step-frame (paused)",
#             "",
#             "[System]",
#             f"  frame #        : {self.frame_no}",
#             f"  mode           : {'PAUSED' if self.paused else 'PLAYING'}",
#             f"  zoom           : {'ON' if self.zoom else 'OFF'}",
#             "",
#         ]

#         if self.mode is SelMode.BAND:
#             lines += [
#                 "[Band selection]",
#                 f"  progress       : {labels[step]}",
#                 f"  band size      : {total}",
#                 "",
#             ]
#         else:
#             lines += [
#                 "[Individual]",
#                 f"  points chosen  : {total}",
#                 "",
#             ]

#         lines += [
#             "[Log]",
#             f"  selections     : {len(self.results)}"
#         ]

#         self.ax_txt.clear()
#         self.ax_txt.axis("off")
#         self.ax_txt.text(0.02, 0.98, "\n".join(lines),
#                          va="top", ha="left", family="monospace", fontsize=9)

#     def _redraw(self) -> None:
#         # right pane ----------------------------------------------------
#         self.ax_img.clear()
#         if self.img is not None:
#             h, w = self.img.shape[:2]
#             self.ax_img.imshow(self.img, origin="upper", zorder=0)
#             self.ax_img.set_xlim(0, w)
#             self.ax_img.set_ylim(h, 0)

#             if self.uv is not None:
#                 self.ax_img.scatter(self.uv[:, 0], self.uv[:, 1],
#                                     s=4, c="lime", alpha=0.6,
#                                     marker="o", zorder=1)

#             if self.band:
#                 pts = self.uv[self.band]
#                 self.ax_img.scatter(pts[:, 0], pts[:, 1],
#                                     s=20, c="black", marker=".",
#                                     zorder=2)

#             if self.indiv:
#                 pts = self.uv[self.indiv]
#                 self.ax_img.scatter(pts[:, 0], pts[:, 1],
#                                     s=40, c="red", marker="x",
#                                     zorder=2)

#             for anchor in (self.a, self.b):
#                 if anchor is not None and anchor < len(self.uv):
#                     u, v = self.uv[anchor]
#                     self.ax_img.scatter(u, v,
#                                         s=60, c="red", marker="x",
#                                         linewidths=2, zorder=3)

#         self.ax_img.axis("off")
#         self.ax_img.set_title("Camera with LiDAR overlay")

#         # left pane -----------------------------------------------------
#         self._update_status_panel()

#         self.fig.canvas.draw_idle()

# # ════════════════════════════════════════════════════════════════════
# # Default CSV writer (only used if you do not provide your own)
# # ════════════════════════════════════════════════════════════════════
# def _default_csv(rows: List[int], path: str = "selected_rows.csv") -> None:
#     import csv, os
#     first = not os.path.exists(path)
#     with open(path, "a", newline="") as f:
#         w = csv.writer(f)
#         if first:
#             w.writerow(["row"])
#         for r in rows:
#             w.writerow([r])
#     print(f"[INFO] wrote {len(rows)} row-ids → {path}")