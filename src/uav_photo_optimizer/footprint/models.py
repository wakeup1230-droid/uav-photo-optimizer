"""
Estimated Ground Footprint models.

Planar footprints are *estimates* on a local ground plane (PLANAR_*). Phase 7 adds
TERRAIN_RASTER (Experimental): boundary rays intersected with a local height field. Neither
is a "true" footprint (pose, lens and terrain model errors remain).
"""

from __future__ import annotations

from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, ConfigDict, Field, field_serializer, field_validator
from shapely.geometry import mapping, shape
from shapely.geometry.base import BaseGeometry

from .lrf import LRFQualityResult


class FootprintMethod(str, Enum):
    PLANAR_LRF_ESTIMATED = "PLANAR_LRF_ESTIMATED"   # horizontal plane through the LRF target
    PLANAR_ESTIMATED = "PLANAR_ESTIMATED"      # horizontal plane from another height source
    TERRAIN_AWARE = "TERRAIN_AWARE"            # reserved (generic terrain method)
    TERRAIN_RASTER = "TERRAIN_RASTER"          # Phase 7: rays ∩ GeoTIFF height field


class FootprintWarning(str, Enum):
    TERRAIN_NOT_ACCOUNTED_FOR = "TERRAIN_NOT_ACCOUNTED_FOR"
    ALTITUDE_TAKEOFF_RELATIVE = "ALTITUDE_TAKEOFF_RELATIVE"   # height = RelativeAltitude
    LOW_CONFIDENCE_HEIGHT = "LOW_CONFIDENCE_HEIGHT"
    LRF_SUSPECT = "LRF_SUSPECT"                               # explicit LRF_RAY_VERTICAL only
    NEIGHBOR_HEIGHT_INTERPOLATED = "NEIGHBOR_HEIGHT_INTERPOLATED"
    NEIGHBOR_HEIGHT_NEAREST = "NEIGHBOR_HEIGHT_NEAREST"
    ALTITUDE_DIFFERENCE_HEIGHT = "ALTITUDE_DIFFERENCE_HEIGHT"   # LRF_LOCAL_PLANE (QA method)
    GROUND_PLANE_USER = "GROUND_PLANE_USER"
    LENS_DISTORTION_IGNORED = "LENS_DISTORTION_IGNORED"
    PRINCIPAL_POINT_ASSUMED = "PRINCIPAL_POINT_ASSUMED"
    OBLIQUE_VIEW = "OBLIQUE_VIEW"                             # off-nadir > 5°
    RTK_NOT_FIXED = "RTK_NOT_FIXED"
    TERRAIN_SURFACE_DERIVED = "TERRAIN_SURFACE_DERIVED"       # terrain not independent
    TERRAIN_PARTIAL_BOUNDARY = "TERRAIN_PARTIAL_BOUNDARY"     # some boundary rays failed
    TERRAIN_FOOTPRINT_REPAIRED = "TERRAIN_FOOTPRINT_REPAIRED" # self-intersecting ring fixed
    TERRAIN_HEIGHT_FLIGHT_OFFSET = "TERRAIN_HEIGHT_FLIGHT_OFFSET"  # camera Z via flight offset
    TERRAIN_FALLBACK_PLANAR = "TERRAIN_FALLBACK_PLANAR"  # terrain failed: planar kept, protected


class PhotoFootprint(BaseModel):
    model_config = ConfigDict(extra="forbid", arbitrary_types_allowed=True)

    photo_id: str
    geometry: BaseGeometry                 # polygon in ``crs`` (GeoJSON when serialized)
    crs: str
    method: FootprintMethod
    confidence: float = Field(ge=0, le=1)  # heuristic score, NOT a probability
    warnings: list[FootprintWarning] = Field(default_factory=list)

    camera_xy: tuple[float, float]                       # camera position in ``crs``
    principal_ground_xy: Optional[tuple[float, float]] = None   # boresight / ground plane
    height_m: float                                      # camera height above ground plane
    height_strategy: str = ""                            # strategy actually applied
    height_source: str = ""                              # HeightQuantity value
    height_confidence: float = Field(1.0, ge=0, le=1)
    lrf_quality: Optional[LRFQualityResult] = None
    corner_xy: list[tuple[float, float]] = Field(default_factory=list)   # image TL, TR, BR, BL
    lens_model: str = "PINHOLE"                          # = distortion_model (compat.)
    lens_mode: str = "PINHOLE"                           # LensMode actually applied
    calibration_source: str = ""                         # Group:Tag of the calibration
    distortion_model: str = "PINHOLE"                    # K6_RATIONAL / BROWN5 / PINHOLE
    projector: str
    provenance: dict[str, Any] = Field(default_factory=dict)   # sources + derived pose

    @field_serializer("geometry")
    def _geom_out(self, geom: BaseGeometry):
        return mapping(geom)

    @field_validator("geometry", mode="before")
    @classmethod
    def _geom_in(cls, value):
        return shape(value) if isinstance(value, dict) else value
