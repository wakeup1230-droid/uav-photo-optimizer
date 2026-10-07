"""
Altitude semantics.

Never treat RelativeAltitude as AGL: it is height above the *take-off point*.
Datums are recorded only when they are known; otherwise UNKNOWN.

    relative_altitude   DJI XMP RelativeAltitude      TAKEOFF_RELATIVE
    absolute_altitude   DJI XMP AbsoluteAltitude      UNKNOWN (likely ellipsoidal for RTK — unverified)
    gps_altitude        EXIF GPS GPSAltitude(+Ref)    UNKNOWN
    lrf_target_abs_alt  DJI XMP LRFTargetAbsAlt       same vertical reference as absolute_altitude
    ground_elevation    DEM / user / LRF target       datum of its source
    agl                 only with a provable source   (None — no DEM yet)

Projection heights are named after what they really are, e.g.
``camera_to_lrf_target_vertical_height`` — never "AGL".
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field


class AltitudeDatum(str, Enum):
    UNKNOWN = "UNKNOWN"
    TAKEOFF_RELATIVE = "TAKEOFF_RELATIVE"
    ELLIPSOIDAL = "ELLIPSOIDAL"
    ORTHOMETRIC = "ORTHOMETRIC"
    AGL = "AGL"


class AltitudeSource(str, Enum):
    DJI_RELATIVE_ALTITUDE = "XMP-drone-dji:RelativeAltitude"
    DJI_ABSOLUTE_ALTITUDE = "XMP-drone-dji:AbsoluteAltitude"
    EXIF_GPS_ALTITUDE = "GPS:GPSAltitude"
    DJI_LRF_TARGET = "XMP-drone-dji:LRFTargetAbsAlt"
    DJI_LRF_DISTANCE = "XMP-drone-dji:LRFTargetDistance"
    GIMBAL_POSE = "XMP-drone-dji:Gimbal{Yaw,Pitch,Roll}Degree"
    NEIGHBOR_LRF = "NEIGHBOR_LRF"                # validated LRF of neighbouring photos
    USER_SUPPLIED = "USER_SUPPLIED"
    DEM = "DEM"                                  # Planned (TerrainProvider)


class HeightStrategy(str, Enum):
    """
    Requested height strategy for PLANAR projection.

    AUTO (Phase 5.1, formal):
        TerrainProvider (Planned) → validated LRF ray height → neighbour LRF interpolation
        (same flight / strip / capture group) → HEIGHT_UNRESOLVED
    RelativeAltitude is used ONLY when TAKEOFF_RELATIVE is requested explicitly.
    """

    AUTO = "AUTO"
    LRF_RAY_VERTICAL = "LRF_RAY_VERTICAL"          # LRFTargetDistance × |ray down component|
    LRF_LOCAL_PLANE = "LRF_LOCAL_PLANE"            # AbsoluteAltitude − LRFTargetAbsAlt (QA / explicit)
    TAKEOFF_RELATIVE = "TAKEOFF_RELATIVE"
    USER_GROUND_ELEVATION = "USER_GROUND_ELEVATION"


class HeightMethod(str, Enum):
    """Height method actually applied to a photo."""

    LRF_RAY_VERTICAL = "LRF_RAY_VERTICAL"
    NEIGHBOR_LRF_INTERPOLATED = "NEIGHBOR_LRF_INTERPOLATED"
    NEIGHBOR_LRF_NEAREST = "NEIGHBOR_LRF_NEAREST"
    LRF_LOCAL_PLANE = "LRF_LOCAL_PLANE"
    TAKEOFF_RELATIVE = "TAKEOFF_RELATIVE"
    USER_GROUND_ELEVATION = "USER_GROUND_ELEVATION"


class HeightStatus(str, Enum):
    RESOLVED = "RESOLVED"
    HEIGHT_UNRESOLVED = "HEIGHT_UNRESOLVED"


class HeightQuantity(str, Enum):
    """The physical meaning of the height used for projection."""

    CAMERA_TO_GROUND_VERTICAL_HEIGHT = "camera_to_ground_vertical_height"      # laser ray
    CAMERA_TO_LRF_TARGET_VERTICAL_HEIGHT = "camera_to_lrf_target_vertical_height"  # altitudes
    RELATIVE_ALTITUDE_ABOVE_TAKEOFF = "relative_altitude_above_takeoff"
    CAMERA_ABOVE_USER_GROUND_PLANE = "camera_above_user_ground_plane"


class ProjectionHeight(BaseModel):
    """Camera height above the assumed ground plane, with provenance."""

    model_config = ConfigDict(extra="forbid")

    height_m: float
    method: HeightMethod
    quantity: HeightQuantity
    sources: list[AltitudeSource]
    vertical_reference: AltitudeDatum = AltitudeDatum.UNKNOWN
    ground_elevation: Optional[float] = None        # in the vertical reference above
    confidence: float = Field(1.0, ge=0, le=1)      # heuristic, not a probability
    neighbours: list[str] = Field(default_factory=list)   # photo_ids used (neighbour methods)
    note: str = ""
