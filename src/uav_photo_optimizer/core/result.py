"""
SelectionResult: the one and only output contract of the Core Engine.

Consumers (CLI / GUI / REST API / external systems) decide what to do with it
(copy files, write a report, return JSON ...). The Core itself never copies.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from ..flight.models import FlightAnalysis
from ..footprint.batch import HeightResolution
from ..footprint.models import PhotoFootprint
from ..overlap.models import OverlapReport
from ..metadata.models import PhotoMetadata
from .config import CandidateSelectionMode


class PhotoReason(str, Enum):
    SELECTED = "SELECTED"
    OUTSIDE_AOI = "OUTSIDE_AOI"
    NO_GPS = "NO_GPS"
    READ_ERROR = "READ_ERROR"
    FOOTPRINT_UNAVAILABLE = "FOOTPRINT_UNAVAILABLE"       # FOOTPRINT mode: cannot estimate
    REDUNDANT_FRONT_OVERLAP = "REDUNDANT_FRONT_OVERLAP"   # Planned (Phase 6)
    REDUNDANT_SIDE_OVERLAP = "REDUNDANT_SIDE_OVERLAP"     # Planned (Phase 6)
    COVERAGE_REQUIRED = "COVERAGE_REQUIRED"               # Planned (Phase 7)


class PhotoRecord(BaseModel):
    """Per-photo decision."""

    model_config = ConfigDict(extra="forbid")

    photo_id: str
    filename: str
    path: Path
    reason: PhotoReason
    message: Optional[str] = None
    metadata: Optional[PhotoMetadata] = None
    footprint: Optional[PhotoFootprint] = None     # FOOTPRINT mode


class AOISummary(BaseModel):
    shapefile: Path
    source_crs: str
    buffer_crs: str
    buffer_m: float
    buffer_area_m2: float


class SelectionResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photos_scanned: int

    candidate_photos: list[PhotoRecord] = Field(default_factory=list)  # inside buffered AOI
    selected_photos: list[PhotoRecord] = Field(default_factory=list)   # final selection
    rejected_photos: list[PhotoRecord] = Field(default_factory=list)   # had GPS, not selected
    no_gps: list[PhotoRecord] = Field(default_factory=list)
    errors: list[PhotoRecord] = Field(default_factory=list)

    buffer_m: float
    front_overlap_target: float
    side_overlap_target: float
    selection_mode: CandidateSelectionMode

    coverage_valid: Optional[bool] = None    # None = not evaluated (Coverage Guard is Phase 7)
    warnings: list[str] = Field(default_factory=list)

    aoi: Optional[AOISummary] = None
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None

    def counts(self) -> dict[str, int]:
        return {
            "photos_scanned": self.photos_scanned,
            "candidate": len(self.candidate_photos),
            "selected": len(self.selected_photos),
            "rejected": len(self.rejected_photos),
            "no_gps": len(self.no_gps),
            "errors": len(self.errors),
        }


class GeometryAnalysis(BaseModel):
    """
    Output of ``UAVPhotoOptimizer.analyse_geometry()`` (Phase 3.1 – 5): footprints, flights /
    strips and overlap. Evaluation only — no photo is selected or removed here.
    """

    model_config = ConfigDict(extra="forbid")

    crs: str
    photos_scanned: int
    footprints: dict[str, PhotoFootprint] = Field(default_factory=dict)
    footprint_unavailable: dict[str, str] = Field(default_factory=dict)   # photo_id -> reason
    height_resolutions: dict[str, HeightResolution] = Field(default_factory=dict)
    flights: FlightAnalysis
    overlap: OverlapReport
    statistics: dict = Field(default_factory=dict)
    warnings: list[str] = Field(default_factory=list)
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None
