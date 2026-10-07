"""
Camera models.

* CameraProfile    — static, documented knowledge about a camera (with provenance).
                     Any field without a reliable source is None. Never guessed.
* CameraIntrinsics — the pinhole model actually used for one photo, resolved
                     metadata-first (see ``camera/intrinsics.py``), with per-field sources.

Note: a "24 mm equivalent" focal length is a 35 mm-format equivalent, NOT the physical
pinhole focal length; it is never stored as ``focal_length_mm``.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from .lens import LensCalibration


class CameraProfile(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    make: str
    model: str
    camera_name: str                       # e.g. "Wide camera"
    image_source: Optional[str] = None     # DJI XMP ImageSource this profile applies to

    sensor_width_mm: Optional[float] = Field(None, gt=0)
    sensor_height_mm: Optional[float] = Field(None, gt=0)
    native_width_px: Optional[int] = Field(None, gt=0)
    native_height_px: Optional[int] = Field(None, gt=0)
    focal_length_mm: Optional[float] = Field(None, gt=0)    # physical, never 35 mm-equivalent
    horizontal_fov_deg: Optional[float] = Field(None, gt=0, lt=180)
    vertical_fov_deg: Optional[float] = Field(None, gt=0, lt=180)

    source: str                            # where the non-None values come from
    verified: bool                         # True = every non-None value checked against `source`
    notes: str = ""


class LensMode(str, Enum):
    AUTO = "AUTO"                          # DewarpDataK6 → DewarpData (Brown5) → Pinhole (warning)
    K6_RATIONAL = "K6_RATIONAL"            # DewarpDataK6 only (falls back to Pinhole)
    BROWN5 = "BROWN5"                      # DewarpData only (falls back to Pinhole)
    PINHOLE = "PINHOLE"                    # CalibratedFocalLength + optical centre, no distortion
    DISTORTION_AWARE = "DISTORTION_AWARE"  # alias of AUTO (kept for compatibility)


class IntrinsicsSource(str, Enum):
    METADATA_DEWARP = "METADATA_DEWARP"                  # XMP DewarpDataK6 / DewarpData
    METADATA_CALIBRATED = "METADATA_CALIBRATED"          # XMP CalibratedFocalLength (px)
    METADATA_FOCAL_MM_PROFILE_SENSOR = "METADATA_FOCAL_MM_PROFILE_SENSOR"
    PROFILE_FOV = "PROFILE_FOV"
    IMAGE_CENTER = "IMAGE_CENTER"                         # principal point assumed at centre


class CameraIntrinsics(BaseModel):
    """
    Camera for one photo: pinhole (focal_px / focal_y_px, cx, cy) plus optional distortion.
    When ``distortion`` is set, recorded pixels are undistorted before ray casting.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    width_px: int = Field(gt=0)
    height_px: int = Field(gt=0)
    focal_px: float = Field(gt=0)            # fx
    focal_y_px: Optional[float] = Field(None, gt=0)   # fy (None = fx)
    cx_px: float                 # principal point, origin = top-left pixel corner
    cy_px: float
    focal_source: str            # IntrinsicsSource value + tag detail
    principal_point_source: str
    profile: Optional[str] = None
    distortion: Optional[LensCalibration] = None
    lens_mode: LensMode = LensMode.PINHOLE

    @property
    def fy(self) -> float:
        return self.focal_y_px or self.focal_px

    @property
    def horizontal_fov_deg(self) -> float:
        return math.degrees(2 * math.atan(self.width_px / 2 / self.focal_px))

    @property
    def vertical_fov_deg(self) -> float:
        return math.degrees(2 * math.atan(self.height_px / 2 / self.fy))
