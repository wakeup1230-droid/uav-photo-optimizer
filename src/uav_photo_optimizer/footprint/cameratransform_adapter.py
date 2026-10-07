"""
CameraTransformProjector — adapter around the optional ``cameratransform`` package.

    pip install "uav-photo-optimizer[footprint]"

The rest of the Core never imports cameratransform; only this module does, lazily.
Orientation mapping (see pose.py / docs/FOOTPRINT_METHOD.md):
    heading_deg = yaw_grid, tilt_deg = 90 + pitch, roll_deg = -roll
CameraTransform world frame: x = east, y = north, z = up; camera at (0, 0, height),
ground plane Z = 0. Rays that do not hit the ground return NaN.
"""

from __future__ import annotations

import numpy as np

from ..camera.models import CameraIntrinsics
from ..core.exceptions import ConfigError
from .base import FootprintProjector
from .pose import GridPose


def _import_cameratransform():
    try:
        import cameratransform
    except ImportError as exc:   # pragma: no cover - depends on environment
        raise ConfigError(
            "CameraTransformProjector needs the optional 'cameratransform' package: "
            'pip install "uav-photo-optimizer[footprint]"') from exc
    return cameratransform


class CameraTransformProjector(FootprintProjector):
    name = "CameraTransformProjector"

    def __init__(self):
        self._ct = _import_cameratransform()

    def _ground_points(self, pose: GridPose, camera: CameraIntrinsics, height_m: float,
                       pixels: np.ndarray) -> np.ndarray:
        ct = self._ct
        projection = ct.RectilinearProjection(
            focallength_x_px=camera.focal_px,
            focallength_y_px=camera.fy,
            image=(camera.width_px, camera.height_px),
            center=(camera.cx_px, camera.cy_px))
        orientation = ct.SpatialOrientation(elevation_m=height_m, pos_x_m=0.0, pos_y_m=0.0,
                                            **pose.cameratransform_orientation())
        cam = ct.Camera(projection, orientation)
        points = np.asarray(cam.spaceFromImage(pixels, Z=0), dtype=float)
        return points[:, :2]
