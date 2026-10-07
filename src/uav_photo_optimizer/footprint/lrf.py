"""
LRF Quality Gate v2 (Phase 5.1).

The laser range ``LRFTargetDistance`` is the primary, direct measurement. The laser ray is the
camera optical axis (coaxial LRF — verified on M4E: median beam/axis angle 0.79°) and its
direction comes from the Pose Convention Adapter (``GridPose.rotation_ned``):

    camera_to_ground_vertical_height = LRFTargetDistance × |down component of the ray|

Gate checks (decide VALID / SUSPECT / INVALID / MISSING):

1. distance validity : LRFStatus == "Normal", distance present and within range limits
2. pose validity     : gimbal pose present, looking down (ray down component ≥ min),
                       roll consistent with 0° / 180° (±tolerance)
3. beam alignment    : angle between the optical axis and the camera → LRF-target direction,
                       where the direction uses the *horizontal* camera / target positions and
                       the *laser* distance only (independent of AbsoluteAltitude / RTK height)
4. footprint         : LRF target inside the estimated footprint (relative tolerance)

There is no separate LRF timestamp in the metadata; a gimbal still moving between the laser
sample and the exposure shows up as beam misalignment (check 3).

QA only (never gates, never lowers height confidence):
* ``altitude_vertical_height_m`` = AbsoluteAltitude − LRFTargetAbsAlt and its difference to
  the laser height (affected by non-fixed RTK, see internal research notes)
* slant residual, altitude-based axis angle
* ``absolute_position_confidence`` from RtkFlag (affects absolute coordinates, not the range)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from pyproj import Geod
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from ..metadata.models import PhotoMetadata
from .pose import GridPose

_GEOD = Geod(ellps="WGS84")

RTK_POSITION_CONFIDENCE = {50: 1.0, 16: 0.5}   # 32–49 float → 0.8; other / missing → None


class LRFQualityStatus(str, Enum):
    VALID = "VALID"
    SUSPECT = "SUSPECT"
    INVALID = "INVALID"
    MISSING = "MISSING"


@dataclass(frozen=True)
class LRFThresholds:
    # 1. distance validity (m) — configurable; data range 92.8–467.8 m
    range_min_m: float = 1.0
    range_max_m: float = 2000.0
    # 2. pose validity
    min_ray_down: float = 0.2              # ray at least ~11.5° below the horizon
    roll_tolerance_deg: float = 15.0       # |roll| within tol of 0° or 180°
    # 3. beam alignment (deg): nadir p99.9 2.4°, oblique p99 5.3°, outlier cluster 10–63°
    beam_valid_deg: float = 5.0
    beam_invalid_deg: float = 10.0
    # 4. footprint: target outside footprint, as a fraction of sqrt(footprint area)
    outside_valid_ratio: float = 0.0
    outside_invalid_ratio: float = 0.10


class LRFQualityResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    status: LRFQualityStatus
    reasons: list[str] = Field(default_factory=list)
    recorded_slant_m: Optional[float] = None
    ray_down_component: Optional[float] = None
    laser_vertical_height_m: Optional[float] = None      # PRIMARY height (LRF_RAY_VERTICAL)
    beam_angle_deg: Optional[float] = None               # laser-based beam alignment (gate)
    outside_footprint_m: Optional[float] = None          # 0 = inside
    outside_ratio: Optional[float] = None
    # QA only
    altitude_vertical_height_m: Optional[float] = None   # AbsoluteAltitude − LRFTargetAbsAlt
    height_difference_m: Optional[float] = None          # altitude − laser
    derived_slant_m: Optional[float] = None
    slant_residual_m: Optional[float] = None
    slant_residual_ratio: Optional[float] = None
    axis_angle_deg: Optional[float] = None               # altitude-based (old gate) angle
    absolute_position_confidence: Optional[float] = None


def _worse(a: LRFQualityStatus, b: LRFQualityStatus) -> LRFQualityStatus:
    order = [LRFQualityStatus.VALID, LRFQualityStatus.SUSPECT, LRFQualityStatus.INVALID]
    return max(a, b, key=order.index)


def lrf_ray(meta: PhotoMetadata) -> np.ndarray:
    """Unit laser ray in true NED (= optical axis via the pose adapter, roll included)."""
    pose = GridPose(meta.gimbal_yaw, meta.gimbal_pitch, meta.gimbal_roll, 0.0)
    return pose.rotation_ned() @ np.array([1.0, 0.0, 0.0])


def horizontal_to_target(meta: PhotoMetadata) -> tuple[float, float]:
    """(horizontal distance m, true azimuth deg) camera → LRF target."""
    az, _, horiz = _GEOD.inv(meta.longitude, meta.latitude,
                             meta.lrf_target_lon, meta.lrf_target_lat)
    return horiz, az


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    cos = float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) or 1.0))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def axis_angle_deg(meta: PhotoMetadata) -> float:
    """QA: optical axis vs camera → target using AbsoluteAltitude − LRFTargetAbsAlt (old gate)."""
    horiz, az = horizontal_to_target(meta)
    vert = meta.absolute_altitude - meta.lrf_target_abs_alt
    t = np.array([horiz * math.cos(math.radians(az)), horiz * math.sin(math.radians(az)), vert])
    return _angle(lrf_ray(meta), t)


def lrf_fields_present(meta: PhotoMetadata) -> bool:
    return (meta.lrf_target_distance is not None and meta.lrf_target_lat is not None
            and meta.lrf_target_lon is not None and meta.has_gps)


def assess_lrf(meta: PhotoMetadata, footprint: Optional[BaseGeometry],
               target_xy: Optional[tuple[float, float]],
               t: LRFThresholds = LRFThresholds()) -> LRFQualityResult:
    if not meta.lrf_status or not lrf_fields_present(meta):
        return LRFQualityResult(status=LRFQualityStatus.MISSING, reasons=["LRF fields missing"])

    r = LRFQualityResult(status=LRFQualityStatus.VALID)
    status, reasons = LRFQualityStatus.VALID, []
    if meta.rtk_flag is not None:
        r.absolute_position_confidence = RTK_POSITION_CONFIDENCE.get(
            meta.rtk_flag, 0.8 if 32 <= meta.rtk_flag <= 49 else None)

    # 1. distance validity
    d = meta.lrf_target_distance
    r.recorded_slant_m = d
    if meta.lrf_status != "Normal":
        status = LRFQualityStatus.INVALID
        reasons.append(f"LRFStatus={meta.lrf_status}")
    if not (t.range_min_m <= d <= t.range_max_m):
        status = LRFQualityStatus.INVALID
        reasons.append(f"distance {d:.1f} m outside [{t.range_min_m}, {t.range_max_m}]")

    # 2. pose validity
    if not meta.has_gimbal_pose:
        r.status, r.reasons = LRFQualityStatus.INVALID, reasons + ["gimbal pose missing"]
        return r
    ray = lrf_ray(meta)
    down = float(ray[2])
    r.ray_down_component = down
    roll_off = min(abs(meta.gimbal_roll), abs(180.0 - abs(meta.gimbal_roll)))
    if down < t.min_ray_down:
        status = LRFQualityStatus.INVALID
        reasons.append(f"ray not looking down enough (down={down:.2f})")
    elif roll_off > t.roll_tolerance_deg:
        status = _worse(status, LRFQualityStatus.SUSPECT)
        reasons.append(f"gimbal roll {meta.gimbal_roll:.1f}° not near 0/180")
    r.laser_vertical_height_m = d * abs(down)

    # 3. beam alignment — horizontal positions + laser distance only
    horiz, az = horizontal_to_target(meta)
    if horiz > d:
        status = LRFQualityStatus.INVALID
        reasons.append(f"target horizontal distance {horiz:.1f} m > laser range {d:.1f} m")
    else:
        vert = math.sqrt(d * d - horiz * horiz)
        target = np.array([horiz * math.cos(math.radians(az)),
                           horiz * math.sin(math.radians(az)), vert])
        r.beam_angle_deg = _angle(ray, target)
        if r.beam_angle_deg > t.beam_invalid_deg:
            status = LRFQualityStatus.INVALID
            reasons.append(f"beam {r.beam_angle_deg:.1f}° off optical axis")
        elif r.beam_angle_deg > t.beam_valid_deg:
            status = _worse(status, LRFQualityStatus.SUSPECT)
            reasons.append(f"beam {r.beam_angle_deg:.1f}° off optical axis")

    # 4. footprint
    if footprint is not None and target_xy is not None:
        p = Point(target_xy)
        out = 0.0 if footprint.covers(p) else footprint.exterior.distance(p)
        r.outside_footprint_m = out
        r.outside_ratio = out / math.sqrt(footprint.area)
        if r.outside_ratio > t.outside_invalid_ratio:
            status = LRFQualityStatus.INVALID
            reasons.append(f"target {out:.1f} m outside footprint")
        elif r.outside_ratio > t.outside_valid_ratio:
            status = _worse(status, LRFQualityStatus.SUSPECT)
            reasons.append(f"target {out:.1f} m outside footprint")

    # QA only
    if meta.absolute_altitude is not None and meta.lrf_target_abs_alt is not None:
        r.altitude_vertical_height_m = meta.absolute_altitude - meta.lrf_target_abs_alt
        r.height_difference_m = r.altitude_vertical_height_m - r.laser_vertical_height_m
        r.derived_slant_m = math.hypot(horiz, r.altitude_vertical_height_m)
        r.slant_residual_m = r.derived_slant_m - d
        r.slant_residual_ratio = abs(r.slant_residual_m) / d
        r.axis_angle_deg = axis_angle_deg(meta)

    r.status, r.reasons = status, reasons
    return r
