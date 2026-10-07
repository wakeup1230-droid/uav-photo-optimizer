"""
Height candidates for PLANAR projection (Phase 5.1 — Height Strategy v2).

    LRF_RAY_VERTICAL           camera_to_ground_vertical_height
                               = LRFTargetDistance × |down component of the laser ray|
                               (direct range measurement; ray from the pose adapter)
    NEIGHBOR_LRF_INTERPOLATED  height from the LRF ground elevation of the previous / next
    NEIGHBOR_LRF_NEAREST       validated photo(s) in the same flight / strip / capture group
                               (see footprint/batch.py)
    LRF_LOCAL_PLANE            AbsoluteAltitude − LRFTargetAbsAlt (QA metric; explicit only)
    TAKEOFF_RELATIVE           RelativeAltitude (not AGL; explicit only — never an AUTO fallback)
    USER_GROUND_ELEVATION      AbsoluteAltitude − user ground elevation

None of these is AGL: terrain is not modelled (TERRAIN_NOT_ACCOUNTED_FOR always applies).
"""

from __future__ import annotations

from typing import Optional

from ..metadata.altitude import (AltitudeDatum, AltitudeSource, HeightMethod, HeightQuantity,
                                 ProjectionHeight)
from ..metadata.models import PhotoMetadata
from .lrf import LRFQualityResult

HEIGHT_CONFIDENCE = {
    "LRF_VALID": 0.9,
    "LRF_SUSPECT": 0.6,            # explicit LRF_RAY_VERTICAL only
    "NEIGHBOR_INTERPOLATED": 0.7,
    "NEIGHBOR_NEAREST": 0.5,
    "LRF_LOCAL_PLANE": 0.6,
    "USER": 0.8,
    "TAKEOFF_RELATIVE": 0.3,
}


def lrf_ray_height(meta: PhotoMetadata, q: LRFQualityResult,
                   confidence: float = HEIGHT_CONFIDENCE["LRF_VALID"]) -> Optional[ProjectionHeight]:
    if q.laser_vertical_height_m is None:
        return None
    ground = (meta.absolute_altitude - q.laser_vertical_height_m
              if meta.absolute_altitude is not None else None)
    return ProjectionHeight(
        height_m=q.laser_vertical_height_m, method=HeightMethod.LRF_RAY_VERTICAL,
        quantity=HeightQuantity.CAMERA_TO_GROUND_VERTICAL_HEIGHT,
        sources=[AltitudeSource.DJI_LRF_DISTANCE, AltitudeSource.GIMBAL_POSE],
        vertical_reference=meta.absolute_altitude_datum, ground_elevation=ground,
        confidence=confidence,
        note="laser range × vertical component of the laser ray (pose adapter); horizontal "
             "plane through the laser ground point")


def neighbour_height(meta: PhotoMetadata, ground_elevation: float, method: HeightMethod,
                     neighbours: list[str]) -> Optional[ProjectionHeight]:
    if meta.absolute_altitude is None:
        return None
    conf = HEIGHT_CONFIDENCE["NEIGHBOR_INTERPOLATED" if method is
                             HeightMethod.NEIGHBOR_LRF_INTERPOLATED else "NEIGHBOR_NEAREST"]
    return ProjectionHeight(
        height_m=meta.absolute_altitude - ground_elevation, method=method,
        quantity=HeightQuantity.CAMERA_TO_GROUND_VERTICAL_HEIGHT,
        sources=[AltitudeSource.DJI_ABSOLUTE_ALTITUDE, AltitudeSource.NEIGHBOR_LRF],
        vertical_reference=meta.absolute_altitude_datum, ground_elevation=ground_elevation,
        confidence=conf, neighbours=neighbours,
        note="ground elevation from validated LRF of neighbouring photos (same flight / strip "
             "/ capture group); AbsoluteAltitude differences over seconds only")


def lrf_local_plane_height(meta: PhotoMetadata) -> Optional[ProjectionHeight]:
    if meta.absolute_altitude is None or meta.lrf_target_abs_alt is None:
        return None
    return ProjectionHeight(
        height_m=meta.absolute_altitude - meta.lrf_target_abs_alt,
        method=HeightMethod.LRF_LOCAL_PLANE,
        quantity=HeightQuantity.CAMERA_TO_LRF_TARGET_VERTICAL_HEIGHT,
        sources=[AltitudeSource.DJI_ABSOLUTE_ALTITUDE, AltitudeSource.DJI_LRF_TARGET],
        vertical_reference=meta.absolute_altitude_datum,
        ground_elevation=meta.lrf_target_abs_alt,
        confidence=HEIGHT_CONFIDENCE["LRF_LOCAL_PLANE"],
        note="altitude difference; affected by non-fixed RTK (QA metric)")


def relative_height(meta: PhotoMetadata) -> Optional[ProjectionHeight]:
    if meta.relative_altitude is None:
        return None
    return ProjectionHeight(
        height_m=meta.relative_altitude, method=HeightMethod.TAKEOFF_RELATIVE,
        quantity=HeightQuantity.RELATIVE_ALTITUDE_ABOVE_TAKEOFF,
        sources=[AltitudeSource.DJI_RELATIVE_ALTITUDE],
        vertical_reference=AltitudeDatum.TAKEOFF_RELATIVE,
        confidence=HEIGHT_CONFIDENCE["TAKEOFF_RELATIVE"],
        note="ground plane at take-off elevation")


def user_height(meta: PhotoMetadata, ground_elevation: Optional[float]
                ) -> Optional[ProjectionHeight]:
    if ground_elevation is None or meta.absolute_altitude is None:
        return None
    return ProjectionHeight(
        height_m=meta.absolute_altitude - ground_elevation,
        method=HeightMethod.USER_GROUND_ELEVATION,
        quantity=HeightQuantity.CAMERA_ABOVE_USER_GROUND_PLANE,
        sources=[AltitudeSource.DJI_ABSOLUTE_ALTITUDE, AltitudeSource.USER_SUPPLIED],
        vertical_reference=meta.absolute_altitude_datum,
        ground_elevation=ground_elevation, confidence=HEIGHT_CONFIDENCE["USER"])
