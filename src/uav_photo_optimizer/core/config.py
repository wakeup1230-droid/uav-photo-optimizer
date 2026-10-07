"""
Run configuration and the single source of truth for parameter validation.

CLI, GUI and REST API must all build these models (or reuse the constants below)
so that there is exactly one set of validation rules.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from ..camera.models import LensMode
from ..metadata.altitude import HeightStrategy

from .exceptions import ConfigError

# ---------------------------------------------------------------------------
# Validation constants (shared by Core / CLI / GUI / API)
# ---------------------------------------------------------------------------

MIN_BUFFER_M = 0.0
DEFAULT_BUFFER_M = 100.0

MIN_OVERLAP = 65.0
MAX_OVERLAP = 95.0
DEFAULT_FRONT_OVERLAP = 80.0
DEFAULT_SIDE_OVERLAP = 70.0

BUFFER_QUAD_SEGS = 64          # arc segments per quarter circle; < 1 cm error at 100 m
PHOTO_EXTENSIONS = frozenset({".jpg", ".jpeg"})
EXIFTOOL_BATCH_SIZE = 500


class CandidateSelectionMode(str, Enum):
    """How candidate photos are selected against the buffered AOI."""

    GPS_POINT = "GPS_POINT"        # photo GPS position covered by buffer (Regression 001)
    FOOTPRINT = "FOOTPRINT"        # Estimated Ground Footprint intersects buffer (Phase 3 POC)


class FootprintProjectorKind(str, Enum):
    CAMERATRANSFORM = "CAMERATRANSFORM"   # optional dependency: pip install .[footprint]
    PLANAR = "PLANAR"                     # NumPy reference implementation (no extra dependency)


class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True)


class AOIConfig(_Model):
    """Area of interest source."""

    shapefile: Optional[Path] = Field(
        None, description="AOI shapefile (.shp). None = the single .shp found in input/shp.")
    buffer_m: float = Field(DEFAULT_BUFFER_M, ge=MIN_BUFFER_M, description="Buffer distance (m).")


class OptimizerConfig(_Model):
    """User-facing optimisation parameters (what GUI / CLI / API expose)."""

    buffer_m: float = Field(DEFAULT_BUFFER_M, ge=MIN_BUFFER_M)
    front_overlap_target: float = Field(DEFAULT_FRONT_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP,
                                        description="Front (along-track) overlap target, %.")
    side_overlap_target: float = Field(DEFAULT_SIDE_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP,
                                       description="Side (cross-track) overlap target, %.")

    @property
    def min_overlap(self) -> float:
        """Hard lower bound for any overlap target (%)."""
        return MIN_OVERLAP


class RunConfig(_Model):
    """Everything the Core Engine needs for one run."""

    aoi: AOIConfig = Field(default_factory=AOIConfig)
    optimizer: OptimizerConfig = Field(default_factory=OptimizerConfig)
    photo_dir: Optional[Path] = Field(None, description="None = <project>/input/photo")
    shp_dir: Optional[Path] = Field(None, description="Used when aoi.shapefile is None. "
                                                      "None = <project>/input/shp")
    base_dir: Optional[Path] = Field(None, description="Project root. None = auto-detect.")
    exiftool_path: Optional[Path] = Field(None, description="Explicit ExifTool executable.")
    selection_mode: CandidateSelectionMode = CandidateSelectionMode.GPS_POINT
    # PLANAR (NumPy) is the default since v1.0: identical to CameraTransform on a reference dataset
    # (Hausdorff 0 m, same plans) and needs no optional dependency
    footprint_projector: FootprintProjectorKind = FootprintProjectorKind.PLANAR
    height_strategy: HeightStrategy = HeightStrategy.AUTO          # FOOTPRINT mode only
    user_ground_elevation: Optional[float] = None                   # HeightStrategy.USER_*
    lens_mode: LensMode = LensMode.AUTO                             # FOOTPRINT mode only

    @model_validator(mode="after")
    def _buffer_consistent(self) -> "RunConfig":
        if self.aoi.buffer_m != self.optimizer.buffer_m:
            raise ValueError(
                f"aoi.buffer_m ({self.aoi.buffer_m}) != optimizer.buffer_m "
                f"({self.optimizer.buffer_m}); use RunConfig.create() to set both")
        return self

    @classmethod
    def create(cls, *, buffer_m: float = DEFAULT_BUFFER_M,
               front_overlap: float = DEFAULT_FRONT_OVERLAP,
               side_overlap: float = DEFAULT_SIDE_OVERLAP,
               shapefile: str | Path | None = None, **kwargs) -> "RunConfig":
        """Convenience constructor; raises ConfigError on invalid parameters."""
        try:
            return cls(
                aoi=AOIConfig(shapefile=shapefile, buffer_m=buffer_m),
                optimizer=OptimizerConfig(buffer_m=buffer_m,
                                          front_overlap_target=front_overlap,
                                          side_overlap_target=side_overlap),
                **kwargs)
        except ValidationError as exc:
            raise ConfigError(str(exc)) from exc
