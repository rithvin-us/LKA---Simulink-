"""Synthetic sign/scale check of the ADAS_2026 bird's-eye pipeline (audit C1, H1).

Renders a ground-truth lane-line probability map through a pinhole camera with a
known pose relative to the lane, runs the team's own IPM + extraction + fit + the
CTE/heading math of lane_detection_node_birdeye.py:484-515, and prints it next to
the truth. The camera (1.2 m high, 10 deg pitch, 1280x720, f = 1109 px) is NOT the
2025 ZED mount: the sign results hold for any camera, the scale numbers do not.

Usage:  python tools/adas2026/verify_signs.py "<path to ADAS_2026/src>"
"""
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
from perception.lane_core.ipm import build_ipm_transform, warp_to_birdeye  # noqa: E402
from perception.lane_core.roi_mask import apply_roi_mask  # noqa: E402
from perception.lane_core.bird_eye_extraction import extract_birdeye_boundaries  # noqa: E402
from perception.lane_core.curves import fit_lane_curve  # noqa: E402
from perception.lane_core.steering_v2 import stanley_steering_deg  # noqa: E402

W, H, F, CX, CY = 1280, 720, 1109.0, 640.0, 360.0
CAM_H, PITCH = 1.2, np.radians(10.0)
LANE_W, LINE_W = 3.5, 0.15


def render(yv, psi, kappa):
    """yv: vehicle lateral position in the lane (+ left); psi: vehicle yaw rel. lane (+ CCW)."""
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    xc, yc = (u - CX) / F, (v - CY) / F
    o = np.array([np.cos(PITCH), 0, -np.sin(PITCH)])
    dn = np.array([-np.sin(PITCH), 0, -np.cos(PITCH)])
    rt = np.array([0, -1.0, 0])
    d = o[None, None, :] + yc[..., None] * dn + xc[..., None] * rt
    ok = d[..., 2] < -1e-6
    t = np.where(ok, CAM_H / np.maximum(-d[..., 2], 1e-9), 0)
    gx, gy = t * d[..., 0], t * d[..., 1]
    ok &= gx < 60
    s = gx * np.cos(psi) - gy * np.sin(psi)
    lat = yv + gx * np.sin(psi) + gy * np.cos(psi) - 0.5 * kappa * s ** 2
    on = ok & ((np.abs(lat - LANE_W / 2) < LINE_W / 2) | (np.abs(lat + LANE_W / 2) < LINE_W / 2))
    prob = np.where(on, 0.95, 0.02).astype(np.float32)
    frame = np.full((H, W, 3), 90, np.uint8)
    frame[on] = 235
    return prob, frame


def pipeline(prob, frame):
    prob = apply_roi_mask(prob)
    wf, wp = warp_to_birdeye(frame, prob, build_ipm_transform(W, H))
    wm = (wp > 0.5).astype(np.uint8)
    wh, ww = wm.shape[:2]
    (ly, lx, lc), (ry, rx, rc) = extract_birdeye_boundaries(wm, wp, wf)
    left = fit_lane_curve(ly, lx, lc) if len(ly) else None
    right = fit_lane_curve(ry, rx, rc) if len(ry) else None
    if left is None or right is None:
        return None
    eyl = float(np.clip(wh - 1, left.y_min, left.y_max))
    eyr = float(np.clip(wh - 1, right.y_min, right.y_max))
    width_px = right.x_at(eyr) - left.x_at(eyl)
    centre_x = (left.x_at(eyl) + right.x_at(eyr)) / 2.0
    cte = (centre_x - ww / 2.0) * LANE_W / width_px
    heading = float(np.degrees(np.arctan((left.slope_at(eyl) + right.slope_at(eyr)) / 2.0)))
    return cte, heading


CASES = [
    ("centred, straight", 0.0, 0.0, 0.0, "none"),
    ("car 0.5 m LEFT of centre", 0.5, 0.0, 0.0, "RIGHT(+)"),
    ("car 0.5 m RIGHT of centre", -0.5, 0.0, 0.0, "LEFT(-)"),
    ("car yawed 3 deg LEFT (CCW)", 0.0, np.radians(3), 0.0, "RIGHT(+)"),
    ("car yawed 3 deg RIGHT (CW)", 0.0, np.radians(-3), 0.0, "LEFT(-)"),
    ("centred, left curve R127.5", 0.0, 0.0, 1 / 127.5, "LEFT(-)"),
    ("centred, right curve R127.5", 0.0, 0.0, -1 / 127.5, "RIGHT(+)"),
]
print(f"{'case':30s} {'cte_m':>7s} {'head_deg':>9s} {'stanley_deg':>11s}  correct steer")
for name, yv, psi, k, need in CASES:
    r = pipeline(*render(yv, psi, k))
    if r is None:
        print(f"{name:30s} lanes not found")
        continue
    cte, head = r
    print(f"{name:30s} {cte:7.3f} {head:9.2f} {stanley_steering_deg(cte, head, 8.33):11.2f}  {need}")
