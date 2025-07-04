#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplest green-rectangle detector:
  • HSV threshold (no contours)
  • Count pixels – if > MIN_PIX, rectangle is present
  • Use mask bounding-box for solvePnP
  • Black out the rectangle pixels after processing
"""

import cv2
import numpy as np
import math
import yaml

# ---------------------------------------------------------------------------
# 0.  INTRINSICS
# ---------------------------------------------------------------------------
with open("/media/zineddine/9D1D-BDBE/Turtlebot4_git/TurtleBot4/"
          "Transformation_matrix/camera_intrinsics.yaml") as f:
    cam = yaml.safe_load(f)
K = np.array(cam["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
D = np.array(cam["distortion_coefficients"]["data"], dtype=np.float32)

# ---------------------------------------------------------------------------
# 1. USER CONSTANTS
# ---------------------------------------------------------------------------
VIDEO_FILE     = "green_segment_20250629_040105.avi"
RECT_W_MM      = 297.0
RECT_H_MM      = 210.0
HSV_LOWER      = (40, 40, 40)
HSV_UPPER      = (90, 255, 255)
MIN_PIX        = 50          # > this many white pixels ⇒ rectangle present
OBJ_PTS = np.array([[0,          0,          0],
                    [RECT_W_MM,  0,          0],
                    [RECT_W_MM,  RECT_H_MM,  0],
                    [0,          RECT_H_MM,  0]], dtype=np.float32)

# ---------------------------------------------------------------------------
def pnp_from_mask(mask):
    """Return (ok, rvec, tvec, boxPts) using bounding box of mask."""
    pts = cv2.findNonZero(mask)
    if pts is None:
        return False, None, None, None
    x, y, w, h = cv2.boundingRect(pts)
    # Build 4 image pts TL,TR,BR,BL
    img_pts = np.array([[x,     y],
                        [x + w, y],
                        [x + w, y + h],
                        [x,     y + h]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, K, D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    return ok, rvec, tvec, img_pts

# ---------------------------------------------------------------------------
cap = cv2.VideoCapture(VIDEO_FILE)
if not cap.isOpened():
    raise IOError("cannot open video")

fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
delay = int(1000 / fps)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.medianBlur(mask, 5)

    # Generate filtered grayscale+highlight image (preview)
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_3ch = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


    white_pix = cv2.countNonZero(mask)
    if white_pix > MIN_PIX:
        ok, rvec, tvec, img_pts = pnp_from_mask(mask)
        if ok:
            # 1. Draw green box
            cv2.polylines(frame, [img_pts.astype(int)], True, (0,255,0), 2)

            # 2. Range from solvePnP
            dist = np.linalg.norm(tvec)

            # 3. Image-based bearing from bounding box
            x, y, w, h = cv2.boundingRect(cv2.findNonZero(mask))
            cx_img = frame.shape[1] * 0.5
            cx_lm  = x + w * 0.5
            dx_px  = cx_lm - cx_img
            fx     = K[0, 0]
            bearing = math.degrees(math.atan2(dx_px, fx))

            # 4. Print and annotate
            print(f"Range = {dist:.0f} mm   Image bearing = {bearing:+.1f}°")
            cv2.putText(frame,
                        f"{dist:.0f} mm  imgBearing={bearing:+.1f}°",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255,0,0), 2)

        

    # Display
    cv2.imshow("mask (B/W)", mask)
    cv2.imshow("frame ", frame)

    if cv2.waitKey(delay) & 0xFF == 27:
        break


cap.release()
cv2.destroyAllWindows()

