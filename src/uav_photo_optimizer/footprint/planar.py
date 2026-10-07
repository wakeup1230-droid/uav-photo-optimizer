"""
PlanarProjector — independent NumPy reference implementation of the DJI convention.

Camera ray for pixel (u, v) in body axes (forward, right, down) = (f, u - cx, v - cy);
rotated to grid NED by R = Rz(yaw) · Ry(pitch) · Rx(roll); intersected with the plane
``down = height``. Used to cross-validate the CameraTransform adapter and as a
dependency-free fallback.
"""

from __future__ import annotations

import numpy as np

from ..camera.models import CameraIntrinsics
from .base import FootprintProjector
from .pose import GridPose


class PlanarProjector(FootprintProjector):
    name = "PlanarProjector"

    def _ground_points(self, pose: GridPose, camera: CameraIntrinsics, height_m: float,
                       pixels: np.ndarray) -> np.ndarray:
        rays_body = np.column_stack([
            np.full(len(pixels), camera.focal_px),
            pixels[:, 0] - camera.cx_px,
            (pixels[:, 1] - camera.cy_px) * camera.focal_px / camera.fy,
        ])
        north, east, down = (pose.rotation_ned() @ rays_body.T)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(down > 1e-9, height_m / down, np.nan)
        return np.column_stack([east * t, north * t])
