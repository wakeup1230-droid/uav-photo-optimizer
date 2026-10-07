"""
Pose Convention Adapter: DJI gimbal pose → projector orientation.

DJI (XMP drone-dji, verified on real data — docs/FOOTPRINT_METHOD.md):
    Gimbal{Yaw,Pitch,Roll}Degree, NED earth frame, Euler order Z-Y-X (yaw, pitch, roll)
    yaw   : 0 = true north, +clockwise (seen from above)
    pitch : 0 = horizon, -90 = nadir
    roll  : about the optical axis; 180 occurs as an equivalent Euler representation

Grid frame used for projection: x = grid east, y = grid north, z = up (target CRS).
``yaw_grid = yaw_true + grid_offset`` where ``grid_offset`` is the grid bearing of true
north at the camera position (meridian convergence), computed numerically with pyproj.

CameraTransform (verified numerically against the reference implementation):
    heading_deg = yaw_grid     tilt_deg = 90 + pitch     roll_deg = -roll
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
from pyproj import CRS, Geod, Transformer

_GEOD = Geod(ellps="WGS84")


@dataclass(frozen=True)
class DJIPose:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    sources: dict = field(default_factory=dict)


@dataclass(frozen=True)
class GridPose:
    """DJI pose expressed in the projection grid (yaw referenced to grid north)."""

    yaw_grid_deg: float
    pitch_deg: float
    roll_deg: float
    grid_offset_deg: float          # grid bearing of true north

    @property
    def off_nadir_deg(self) -> float:
        return 90.0 + self.pitch_deg if self.pitch_deg >= -90 else -90.0 - self.pitch_deg

    def cameratransform_orientation(self) -> dict:
        return {"heading_deg": self.yaw_grid_deg, "tilt_deg": 90.0 + self.pitch_deg,
                "roll_deg": -self.roll_deg}

    def rotation_ned(self) -> np.ndarray:
        """Body(forward, right, down) → grid NED, R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
        y, p, r = np.radians([self.yaw_grid_deg, self.pitch_deg, self.roll_deg])
        rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        return rz @ ry @ rx


@lru_cache(maxsize=16)
def wgs84_to(crs: str) -> Transformer:
    """Cached EPSG:4326 → ``crs`` transformer (lon/lat order)."""
    return Transformer.from_crs("EPSG:4326", CRS(crs), always_xy=True)


def grid_north_offset(lon: float, lat: float, crs: CRS | str) -> float:
    """Grid bearing (deg, clockwise from grid north) of the true-north direction at (lon, lat)."""
    crs = CRS(crs)
    if crs.is_geographic:
        return 0.0
    to_grid = wgs84_to(crs.to_wkt())
    lon2, lat2, _ = _GEOD.fwd(lon, lat, 0.0, 100.0)
    x1, y1 = to_grid.transform(lon, lat)
    x2, y2 = to_grid.transform(lon2, lat2)
    return math.degrees(math.atan2(x2 - x1, y2 - y1))


def to_grid_pose(pose: DJIPose, lon: float, lat: float, crs: CRS | str) -> GridPose:
    offset = grid_north_offset(lon, lat, crs)
    return GridPose(yaw_grid_deg=pose.yaw_deg + offset, pitch_deg=pose.pitch_deg,
                    roll_deg=pose.roll_deg, grid_offset_deg=offset)
