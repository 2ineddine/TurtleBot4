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
# interactive_window_debug.py
# ──────────────────────────────────────────────────────────────────────────────
# A verbosely-instrumented variant of LiDARCameraWindow so we can see every
# state transition that might explain why “resume” sometimes *looks* dead.
# interactive_window.py – cleaned‑up version with reliable play/pause toggle
# ---------------------------------------------------------------------------
# Key fixes:
#   • Swallow key auto‑repeat events so holding "p" does **not** bounce
#     between paused/playing.
#   • push_frame no longer early‑returns when paused; outer loop handles that.
#   • Tiny on‑screen frame counter helps confirm playback.
#
# If you want the super‑verbose logging you can drop back in the IW‑DEBUG prints
# from the debug copy we used earlier.
# interactive_window.py – robust play/pause, single‑toggle per key press
# ---------------------------------------------------------------------------
# Highlights vs. the original:
#   • **Single‑toggle on key‑release** – regardless of OS auto‑repeat.
#   • Auto‑repeat suppression for Qt (using isAutoRepeat) *and* time‑based
#     debounce fallback for back‑ends that don’t expose it.
#   • push_frame never early‑returns on pause; outer loop gates playback.
#   • Subtle frame counter overlay so you can see movement.
#
# Drop this file in place of the old `interactive_window.py` and keep the
# current `main.py` that uses a while‑loop with a pause gate.
# interactive_window.py – robust play/pause, single‑toggle per key press
# ---------------------------------------------------------------------------
# Highlights vs. the original:
#   • **Single‑toggle on key‑release** – regardless of OS auto‑repeat.
#   • Auto‑repeat suppression for Qt (using isAutoRepeat) *and* time‑based
#     debounce fallback for back‑ends that don’t expose it.
#   • push_frame never early‑returns on pause; outer loop gates playback.
#   • Subtle frame counter overlay so you can see movement.
#
# Drop this file in place of the old `interactive_window.py` and keep the
# current `main.py` that uses a while‑loop with a pause gate.
#!/usr/bin/env python3
# interactive_window.py – reliable play/pause, band/point selection, status UI
# -----------------------------------------------------------------------------
# from __future__ import annotations
# from typing import List, Tuple, Optional, Dict, Any
# from dataclasses import dataclass
# from enum import Enum
# import time

# import numpy as np
# import matplotlib.pyplot as plt
# import cv2

# # ═════════════════════════ chess-board helper ════════════════════════════════
# class ChessboardDetector:
#     """Detect a calibration chessboard in an RGB image and fit its 3-D plane."""
#     def __init__(self, size: Tuple[int, int] = (7, 10), square: float = 0.025):
#         self.size = size
#         obj = np.zeros((size[0] * size[1], 3), np.float32)
#         obj[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
#         self.obj_pts = obj * square

#     def detect_and_fit_plane(
#         self, img_rgb: np.ndarray, K: np.ndarray, D: np.ndarray
#     ) -> Optional[Tuple[float, float, float, float]]:
#         g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

#         ok, c = cv2.findChessboardCornersSB(
#             g, self.size, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
#         )
#         if not ok:
#             ok, c = cv2.findChessboardCorners(
#                 g, self.size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
#             )
#         if not ok or c.shape[0] < 4:
#             return None

#         cv2.cornerSubPix(
#             g,
#             c,
#             (11, 11),
#             (-1, -1),
#             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3),
#         )

#         ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c, K, D.reshape(-1, 1))
#         if not ok:
#             return None

#         R, _ = cv2.Rodrigues(rvec)
#         n = (R @ np.array([0, 0, 1])).ravel()
#         n /= np.linalg.norm(n)
#         d = -float(n @ tvec.ravel())
#         return (*n, d)

# # ═════════════════════════ small containers ══════════════════════════════════
# class SelMode(Enum):
#     BAND = "band"
#     INDIV = "individual"

# @dataclass
# class Selection:
#     rows: List[int]
#     plane: Tuple[float, float, float, float]
#     frame: int
#     stamp: Optional[float] = None
#     meta: Dict[str, Any] | None = None

# # ═════════════════════════ main window ═══════════════════════════════════════
# class LiDARCameraWindow:
#     """
#     Matplotlib window that overlays LiDAR points on camera frames and lets the
#     user pick bands or individual points.  'p' toggles play/pause once per
#     physical key press; 'q' must be double-tapped (within 1 s) to quit.
#     """

#     # class-level flags for debouncing
#     _p_down: bool = False                 # current physical key-down state
#     _last_toggle: float = 0.0             # last time we flipped pause

#     def __init__(
#         self,
#         K: np.ndarray,
#         D: np.ndarray,
#         mode: SelMode = SelMode.BAND,
#         chess_size: Tuple[int, int] = (7, 10),
#         square: float = 0.025,
#     ):
#         self.K, self.D = K, D
#         self.detector = ChessboardDetector(chess_size, square)
#         self.mode = mode

#         # ――― figure ―――
#         self.fig, (self.ax_txt, self.ax_img) = plt.subplots(
#             1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1, 2.5]}
#         )
#         self.ax_img.set_aspect("equal")
#         self.fig.canvas.mpl_connect("key_press_event",   self._on_key_down)
#         self.fig.canvas.mpl_connect("key_release_event", self._on_key_up)
#         self.fig.canvas.mpl_connect("button_press_event", self._on_click)

#         # ――― dynamic data ―――
#         self.img: np.ndarray | None = None
#         self.uv:  np.ndarray | None = None
#         self.scan_idx: np.ndarray | None = None
#         self.frame_no = 0
#         self.stamp: float | None = None

#         # ――― UI state ―――
#         self.paused = False
#         self.zoom   = False
#         self.quit   = False

#                 # UI state ----------------------------------------------------------
#         self.paused = False
#         self.zoom   = False
#         self.quit   = False

#         # keep the most-recent zoom rectangle when user exits zoom mode
#         self._frozen_limits: tuple[tuple[float, float], tuple[float, float]] | None = None


#         # ――― selection state ―――
#         self.a: int | None = None
#         self.b: int | None = None
#         self.band:  list[int] = []
#         self.indiv: list[int] = []

#         # ――― results ―――
#         self.results: list[Selection] = []
#         self.quit_requested = False

#         self._redraw()

#     # ───────────────────────── public helpers ────────────────────────────────
#     def push_frame(
#         self,
#         img_rgb: np.ndarray,
#         scan_idx: np.ndarray,
#         uv: np.ndarray,
#         timestamp: float | None = None,
#     ) -> bool:
#         """Present a new frame. Return False if the user has asked to quit."""
#         if self.quit:
#             return False
#         self._reset()
#         self._frozen_limits = None 
#         self.img      = img_rgb
#         self.scan_idx = scan_idx
#         self.uv       = uv
#         self.frame_no += 1
#         self.stamp    = timestamp
#         self._redraw()
#         return True

#     def result(self) -> list[Selection]:
#         plt.show()          # blocks until window is closed
#         return self.results

#     # ───────────────────────── key handling ─────────────────────────────────
#     def _on_key_down(self, e):
#         # Ignore auto-repeat if backend exposes .isAutoRepeat()
#         if hasattr(e, "guiEvent") and getattr(e.guiEvent, "isAutoRepeat", lambda: False)():
#             return
#         if e.key == "p":
#             LiDARCameraWindow._p_down = True
#         elif e.key == "z" and self.paused:
#             prev_zoom_state = self.zoom          # state *before* toggle
#             self.zoom = not self.zoom            # flip the flag

#             if prev_zoom_state and not self.zoom:
#                 # we are LEAVING zoom mode → freeze current limits
#                 self._frozen_limits = (
#                     self.ax_img.get_xlim(),
#                     self.ax_img.get_ylim()
#                 )
#             elif (not prev_zoom_state) and self.zoom:
#                 # entering zoom mode → forget any old rectangle
#                 self._frozen_limits = None

#             tb = getattr(self.fig.canvas, "toolbar", None)
#             if tb:
#                 tb.zoom()
#             self._redraw()

#         elif e.key == "r":
#             # OLD behaviour: simply clear the current anchors / band / points
#             self._reset()
#             self._redraw()

#         elif e.key == "q":
#             t = time.time()
#             if getattr(self, "_quit_armed", 0) and (t - self._quit_armed) < 1.0:
#                 self.quit = True
#                 plt.close(self.fig)
#             else:
#                 print("[press q again within 1 s to quit]")
#                 self._quit_armed = t
#         elif e.key == " " and self.paused:
#             # single-step while paused; outer loop will pause again
#             self.paused = False

#     def _on_key_up(self, e):
#         if e.key == "p" and LiDARCameraWindow._p_down:
#             LiDARCameraWindow._p_down = False
#             now = time.time()
#             # 120 ms debounce so very fast taps don't toggle twice
#             if now - LiDARCameraWindow._last_toggle > 0.12:
#                 self.paused = not self.paused
#                 LiDARCameraWindow._last_toggle = now
#                 if not self.paused:
#                     self.zoom = False
#                     tb = getattr(self.fig.canvas, "toolbar", None)
#                     if tb:
#                         tb.home()
#                 self._redraw()

#     # ───────────────────────── mouse handling ───────────────────────────────
#     def _on_click(self, e):
#         if (
#             e.button != 1
#             or not self.paused
#             or self.zoom
#             or e.inaxes != self.ax_img
#             or self.uv is None
#         ):
#             return
#         # nearest LiDAR point
#         d2  = (self.uv[:, 0] - e.xdata) ** 2 + (self.uv[:, 1] - e.ydata) ** 2
#         row = int(np.argmin(d2))
#         if self.mode is SelMode.BAND:
#             self._band_click(row)
#         else:
#             self._indiv_click(row)
#         self._redraw()

#     # ───────────────────────── selection helpers ────────────────────────────
#         # ───────────────────────── selection helpers ────────────────────────────
#     def _band_click(self, row: int):
#         if self.a is None:
#             self.a = row
#             return

#         if self.b is None:
#             self.b = row
#             self._build_band()
#             self._record()           # This calls _reset() internally now
#             self._redraw()
#             return

#         # Third click starts fresh
#         self._reset()
#         self.a = row
#         self._redraw()


#     def _indiv_click(self, row: int):
#         if row in self.indiv:
#             self.indiv.remove(row)
#         else:
#             self.indiv.append(row)

#     def _build_band(self):
#         """Fill self.band using original scan indexes, handling 0-deg wrap."""
#         if self.scan_idx is None or self.a is None or self.b is None:
#             return
#         idx_a, idx_b = int(self.scan_idx[self.a]), int(self.scan_idx[self.b])
#         total_bins   = int(self.scan_idx.max()) + 1
#         if abs(idx_a - idx_b) <= total_bins / 2:
#             lo, hi = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= lo) & (self.scan_idx <= hi)
#         else:                                   # wrap through 0°
#             lo, hi = sorted((idx_a, idx_b))
#             mask = (self.scan_idx >= hi) | (self.scan_idx <= lo)
#         self.band = np.where(mask)[0].tolist()

#     def _reset(self):
#         self.a = self.b = None
#         self.band.clear()
#         self.indiv.clear()
#         print("IW-DEBUG  reset() called")   # ← add this line

#     def _record(self) -> None:
#         """Record the current selection as a Selection object."""
#         if self.img is None or self.uv is None:
#             return
        
#         rows = self.band if self.mode is SelMode.BAND else self.indiv
#         if not rows:
#             print("DEBUG  record(): rows empty")
#             return
        
#         plane = self.detector.detect_and_fit_plane(self.img, self.K, self.D) \
#                 or (0.0, 0.0, 0.0, 0.0)
        
#         # Create and append the selection - ONLY ONCE
#         selection = Selection(
#             rows=rows.copy(),
#             plane=plane,
#             frame=self.frame_no,
#             stamp=self.stamp,
#         )
        
#         self.results.append(selection)
#         print(f"DEBUG  record(): saved selection with {len(rows)} points, total = {len(self.results)}")
        
#         # Clear the current selection after recording
#         self._reset()
#     # ───────────────────────── drawing routine ──────────────────────────────
#     def _redraw(self):
#         """Redraw both panes unconditionally (blocking)."""
#         # ―― right: image & overlays ―――――――――――――――――――――――――――――――――――――
#         self.ax_img.clear()
#         if self.img is not None:
#             h, w = self.img.shape[:2]
#             self.ax_img.imshow(self.img, origin="upper", zorder=0)
#             if self._frozen_limits is not None and self.paused:
#                 self.ax_img.set_xlim(*self._frozen_limits[0])
#                 self.ax_img.set_ylim(*self._frozen_limits[1])
#             else:
#                 self.ax_img.set_xlim(0, w)
#                 self.ax_img.set_ylim(h, 0)

#             # LiDAR points
#             if self.uv is not None:
#                 self.ax_img.scatter(
#                     self.uv[:, 0], self.uv[:, 1], s=4, c="lime",
#                     alpha=0.6, marker="o", zorder=1
#                 )
#             # current band / individual points
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
#             # anchor marks
#             for r in (self.a, self.b):
#                 if r is not None and r < len(self.uv):
#                     u, v = self.uv[r]
#                     self.ax_img.scatter(u, v,
#                                         s=60, c="red", marker="x",
#                                         linewidths=2, zorder=3)
#             # frame counter overlay
#             self.ax_img.text(
#                 0.98, 0.02, f"{self.frame_no}", transform=self.ax_img.transAxes,
#                 ha="right", va="bottom", color="yellow", size=24, weight="bold",
#                 zorder=4
#             )
#         self.ax_img.axis("off")
#         self.ax_img.set_title("Camera with LiDAR overlay")

#         # ―― left: status text ――――――――――――――――――――――――――――――――――――――――――
#         self.ax_txt.clear()
#         self.ax_txt.axis("off")
#         sel_size = len(self.band) if self.mode is SelMode.BAND else len(self.indiv)
#         status_lines = [
#             "─────────  STATUS  ─────────",
#             f"frame     : {self.frame_no}",
#             f"paused    : {self.paused}",
#             f"zoom      : {self.zoom}",
#             f"mode      : {self.mode.value}",
#             f"selection : {sel_size} pts",
#             f"saved     : {len(self.results)}",
#             "",
#             "keys:",
#             "  p – play/pause",
#             "  z – zoom (paused)",
#             "  r – reset sel.",
#             "  space – step",
#             "  q q – quit",
#         ]
#         self.ax_txt.text(
#             0.02, 0.98, "\n".join(status_lines),
#             va="top", ha="left", family="monospace", fontsize=10
#         )

#         # ―― final paint ――――――――――――――――――――――――――――――――――――――――――――――
#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()

############################################################################################ here 
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
#@@@@@@@@@@@@@@@@@@@@
from __future__ import annotations
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import time
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathlib import Path
import csv
from typing import List, Tuple, Union

# ═════════════════════════ chess-board helper ════════════════════════════════
class ChessboardDetector:
    """Detect a calibration chessboard in an RGB image and fit its 3-D plane."""
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
        if not ok or c.shape[0] < 4:
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

# ═════════════════════════ small containers ══════════════════════════════════
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

# ═════════════════════════ main window ═══════════════════════════════════════
class LiDARCameraWindow:
    """
    Matplotlib window that overlays LiDAR points on camera frames and lets the
    user pick bands or individual points. 'p' toggles play/pause once per
    physical key press; 'q' must be double-tapped (within 1 s) to quit.
    'e' saves the current selection.
    """

    # class-level flags for debouncing
    _p_down: bool = False                 # current physical key-down state
    _last_toggle: float = 0.0             # last time we flipped pause

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
        self.lidar2d_pts: List[Tuple[int, Tuple[float, float, float]]] = []        # ――― figure ―――
        self.fig, (self.ax_txt, self.ax_img) = plt.subplots(
            1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1, 2.5]}
        )
        self.ax_img.set_aspect("equal")
        self.fig.canvas.mpl_connect("key_press_event",   self._on_key_down)
        self.fig.canvas.mpl_connect("key_release_event", self._on_key_up)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        # ――― dynamic data ―――
        self.img: np.ndarray | None = None
        self.uv:  np.ndarray | None = None
        self.scan_idx: np.ndarray | None = None
        self.frame_no = 0
        self.stamp: float | None = None

        # ――― UI state ―――
        self.paused = False
        self.zoom   = False
        self.quit   = False

        # keep the most-recent zoom rectangle when user exits zoom mode
        self._frozen_limits: tuple[tuple[float, float], tuple[float, float]] | None = None

        # ――― selection state ―――
        self.a: int | None = None
        self.b: int | None = None
        self.band:  list[int] = []
        self.indiv: list[int] = []
        
        # ――― selection ready state ―――
        self.selection_ready = False  # True when selection is complete and ready to save

        # ――― results ―――
        self.results: list[Selection] = []
        self.quit_requested = False

        # ――― color palette for selections ―――
        self.selection_colors = ['orange', 'cyan', 'magenta', 'yellow', 'purple', 'brown']
        self.current_color_idx = 0

        self._redraw()

    # ───────────────────────── public helpers ────────────────────────────────
    def push_frame(
        self,
        index_lidar2d : List[Tuple[int, Tuple[float, float, float]]],
        img_rgb: np.ndarray,
        scan_idx: np.ndarray,
        uv: np.ndarray,
        timestamp: float | None = None,
        
    ) -> bool:
        """Present a new frame. Return False if the user has asked to quit."""
        if self.quit:
            return False
        self._reset()
        self._frozen_limits = None 
        self.img      = img_rgb
        self.scan_idx = scan_idx
        self.uv       = uv
        self.lidar2d_pts = index_lidar2d
        self.frame_no += 1
        self.stamp    = timestamp
        self._redraw()
        return True

    def result(self) -> list[Selection]:
        try:
            if not self.quit and plt.fignum_exists(self.fig.number):
                plt.show()
        except Exception as e:
            print(f"Error in result(): {e}")
        finally:
            return self.results  # Always return results

    # ───────────────────────── key handling ─────────────────────────────────
    def _on_key_down(self, e):
        # Ignore auto-repeat if backend exposes .isAutoRepeat()
        if hasattr(e, "guiEvent") and getattr(e.guiEvent, "isAutoRepeat", lambda: False)():
            return
        if e.key == "p":
            LiDARCameraWindow._p_down = True
        elif e.key == "z" and self.paused:
            prev_zoom_state = self.zoom          # state *before* toggle
            self.zoom = not self.zoom            # flip the flag

            if prev_zoom_state and not self.zoom:
                # we are LEAVING zoom mode → freeze current limits
                self._frozen_limits = (
                    self.ax_img.get_xlim(),
                    self.ax_img.get_ylim()
                )
            elif (not prev_zoom_state) and self.zoom:
                # entering zoom mode → forget any old rectangle
                self._frozen_limits = None

            tb = getattr(self.fig.canvas, "toolbar", None)
            if tb:
                tb.zoom()
            self._redraw()

        elif e.key == "r":
            # Clear the current anchors / band / points
            self._reset()
            self._redraw()

        elif e.key == "e" and self.paused:
            # Save current selection if ready
            if self.selection_ready:
                self._record()
                self._redraw()
            else:
                print("No selection ready to save. Complete your selection first.")

        elif e.key == "q":
            t = time.time()
            if getattr(self, "_quit_armed", 0) and (t - self._quit_armed) < 1.0:
                self.quit = True
                plt.close(self.fig)
            else:
                print("[press q again within 1 s to quit]")
                self._quit_armed = t
        elif e.key == " " and self.paused:
            # single-step while paused; outer loop will pause again
            self.paused = False

    def _on_key_up(self, e):
        if e.key == "p" and LiDARCameraWindow._p_down:
            LiDARCameraWindow._p_down = False
            now = time.time()
            # 120 ms debounce so very fast taps don't toggle twice
            if now - LiDARCameraWindow._last_toggle > 0.12:
                self.paused = not self.paused
                LiDARCameraWindow._last_toggle = now
                if not self.paused:
                    self.zoom = False
                    tb = getattr(self.fig.canvas, "toolbar", None)
                    if tb:
                        tb.home()
                self._redraw()

    # ───────────────────────── mouse handling ───────────────────────────────
    def _on_click(self, e):
        if (
            e.button != 1
            or not self.paused
            or self.zoom
            or e.inaxes != self.ax_img
            or self.uv is None
        ):
            return
        # nearest LiDAR point
        d2  = (self.uv[:, 0] - e.xdata) ** 2 + (self.uv[:, 1] - e.ydata) ** 2
        row = int(np.argmin(d2))
        if self.mode is SelMode.BAND:
            self._band_click(row)
        else:
            self._indiv_click(row)
        self._redraw()

    # ───────────────────────── selection helpers ────────────────────────────
    def _band_click(self, row: int):
        """
        Modified logic - no auto-save:
        * first click  → remember anchor A
        * second click → remember anchor B, build the band, mark as ready
        * third click  → starts a fresh band
        """
        if self.a is None:
            # first anchor
            self.a = row
            self.selection_ready = False
            print("First anchor set. Click second anchor to complete band selection.")
            return

        if self.b is None:
            # second anchor → build band, mark as ready (but don't save yet)
            self.b = row
            self._build_band()
            self.selection_ready = True
            print(f"Band selection complete with {len(self.band)} points. Press 'e' to save.")
            return

        # third click → start a brand-new band
        self._reset()
        self.a = row
        self.selection_ready = False
        print("New band started. Click second anchor to complete selection.")

    def _indiv_click(self, row: int):
        if row in self.indiv:
            self.indiv.remove(row)
        else:
            self.indiv.append(row)
        
        # Mark as ready if we have at least one point selected
        self.selection_ready = len(self.indiv) > 0
        if self.selection_ready:
            print(f"Individual selection: {len(self.indiv)} points. Press 'e' to save.")

    def _build_band(self):
        """Fill self.band using original scan indexes, handling 0-deg wrap."""
        if self.scan_idx is None or self.a is None or self.b is None:
            return
        idx_a, idx_b = int(self.scan_idx[self.a]), int(self.scan_idx[self.b])
        total_bins   = int(self.scan_idx.max()) + 1
        if abs(idx_a - idx_b) <= total_bins / 2:
            lo, hi = sorted((idx_a, idx_b))
            mask = (self.scan_idx >= lo) & (self.scan_idx <= hi)
        else:                                   # wrap through 0°
            lo, hi = sorted((idx_a, idx_b))
            mask = (self.scan_idx >= hi) | (self.scan_idx <= lo)
        self.band = np.where(mask)[0].tolist()

    def _reset(self):
        self.a = self.b = None
        self.band.clear()
        self.indiv.clear()
        self.selection_ready = False
        print("Selection reset")



    def export_selected_points(self, selections: List[Selection],  point_pairs: List[Tuple[int, Tuple[float, float, float]]], csv_path: Union[str, Path] = "selected_points.csv",
    ) -> None:
        """
        Write one row per LiDAR point listed in every Selection.
        Appends to the CSV (does not overwrite), writing the header only once.

        CSV columns: scan_idx, x, y, z, a, b, c, d
        """
        # Build lookup: scan_idx → (x, y, z)
        xyz_lut = { idx: xyz for idx, xyz in point_pairs }

        # Resolve to an absolute path in the current working directory
        csv_path = Path.cwd() / Path(csv_path)

        # Determine whether we need to write the header
        new_file = not csv_path.exists() or csv_path.stat().st_size == 0

        # Open for append
        with csv_path.open("a", newline="") as f:
            writer = csv.writer(f)
            if new_file:
                # Write header row once
                writer.writerow(["scan_idx", "x", "y", "z", "a", "b", "c", "d"])

            # Write one row per selected LiDAR point in each Selection
            total_rows = 0
            for sel in selections:
                a, b, c, d = sel.plane
                for idx in sel.rows:
                    xyz = xyz_lut.get(idx)
                    if xyz is None:
                        continue
                    x, y, z = xyz
                    writer.writerow([idx, x, y, z, a, b, c, d])
                    total_rows += 1

        print(f"[INFO] appended {total_rows} rows to {csv_path.resolve()}")


    def _record(self) -> None:
        """Record the current selection as a Selection object."""
        if self.img is None or self.uv is None:
            return
        
        rows = self.band if self.mode is SelMode.BAND else self.indiv
        if not rows:
            print("DEBUG  record(): rows empty")
            return
        
        plane = self.detector.detect_and_fit_plane(self.img, self.K, self.D) \
                or (0.0, 0.0, 0.0, 0.0)
        
        # Create and append the selection
        selection = Selection(
            rows=rows.copy(),
            plane=plane,
            frame=self.frame_no,
            stamp=self.stamp,
        )
        self.export_selected_points([selection],self.lidar2d_pts)
        
        #self.results.append(selection)
        print(f"Selection saved with {len(rows)} points, total saved = {len(self.results)}")
        
        # Move to next color for future selections
        self.current_color_idx = (self.current_color_idx + 1) % len(self.selection_colors)
        
        # Clear the current selection after recording
        self._reset()

    # ───────────────────────── drawing routine ──────────────────────────────
    def _redraw(self):
        """Redraw both panes unconditionally (blocking)."""
        # ―― right: image & overlays ―――――――――――――――――――――――――――――――――――――
        self.ax_img.clear()
        if self.img is not None:
            h, w = self.img.shape[:2]
            self.ax_img.imshow(self.img, origin="upper", zorder=0)
            if self._frozen_limits is not None and self.paused:
                self.ax_img.set_xlim(*self._frozen_limits[0])
                self.ax_img.set_ylim(*self._frozen_limits[1])
            else:
                self.ax_img.set_xlim(0, w)
                self.ax_img.set_ylim(h, 0)

            # LiDAR points (base layer)
            if self.uv is not None:
                self.ax_img.scatter(
                    self.uv[:, 0], self.uv[:, 1], s=4, c="lime",
                    alpha=0.6, marker="o", zorder=1
                )
            
            # Current selection (colored and prominent)
            current_color = self.selection_colors[self.current_color_idx]
            
            if self.band:
                pts = self.uv[self.band]
                self.ax_img.scatter(pts[:, 0], pts[:, 1],
                                    s=25, c=current_color, marker="o",
                                    alpha=0.8, edgecolors='black', linewidths=0.5,
                                    zorder=2)
            if self.indiv:
                pts = self.uv[self.indiv]
                self.ax_img.scatter(pts[:, 0], pts[:, 1],
                                    s=40, c=current_color, marker="s",
                                    alpha=0.8, edgecolors='black', linewidths=1,
                                    zorder=2)
            
            # Anchor marks (always red X's for visibility)
            for r in (self.a, self.b):
                if r is not None and r < len(self.uv):
                    u, v = self.uv[r]
                    self.ax_img.scatter(u, v,
                                        s=80, c="red", marker="x",
                                        linewidths=3, zorder=3)
            
            # Frame counter overlay
            self.ax_img.text(
                0.98, 0.02, f"{self.frame_no}", transform=self.ax_img.transAxes,
                ha="right", va="bottom", color="yellow", size=24, weight="bold",
                zorder=4
            )
        self.ax_img.axis("off")
        self.ax_img.set_title("Camera with LiDAR overlay")

        # ―― left: status text ――――――――――――――――――――――――――――――――――――――――――
        self.ax_txt.clear()
        self.ax_txt.axis("off")
        sel_size = len(self.band) if self.mode is SelMode.BAND else len(self.indiv)
        ready_status = "✓ READY" if self.selection_ready else "building..."
        
        status_lines = [
            "─────────  STATUS  ─────────",
            f"frame     : {self.frame_no}",
            f"paused    : {self.paused}",
            f"zoom      : {self.zoom}",
            f"mode      : {self.mode.value}",
            f"selection : {sel_size} pts ({ready_status})",
            f"saved     : {len(self.results)}",
            f"color     : {self.selection_colors[self.current_color_idx]}",
            "",
            "keys:",
            "  p – play/pause",
            "  z – zoom (paused)",
            "  r – reset sel.",
            "  e – save selection",
            "  space – step",
            "  q q – quit",
        ]
        self.ax_txt.text(
            0.02, 0.98, "\n".join(status_lines),
            va="top", ha="left", family="monospace", fontsize=10
        )

        # ―― final paint ――――――――――――――――――――――――――――――――――――――――――――――
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()