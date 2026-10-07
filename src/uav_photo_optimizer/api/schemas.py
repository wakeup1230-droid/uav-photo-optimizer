"""
REST API v1 contract — core scope only.

Request: AOI source, photo source, output, buffer, front / side overlap.
Result: candidate / selected / removed counts, reduction, selected and removed photo lists.

Research details (LRF, terrain, visual matching, lens, ray solver) are NOT part of API v1;
they stay in internal / debug models. Validation limits come from core.config so API, CLI,
GUI and Core share one rule set. See docs/API_DESIGN.md.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from ..core.config import (DEFAULT_BUFFER_M, DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP,
                           MAX_OVERLAP, MIN_BUFFER_M, MIN_OVERLAP)
from ..core.selection import PhotoItem, SelectionRequest

API_VERSION = "v1"


class JobStatus(str, Enum):
    QUEUED = "queued"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class CreateJobRequest(BaseModel):
    """POST /api/v1/jobs — paths are on the machine running the server."""

    model_config = ConfigDict(extra="forbid")

    aoi_shapefile: str = Field(..., description="AOI Shapefile (.shp)")
    photo_dir: str = Field(..., description="Photo folder, searched recursively")
    output_dir: Optional[str] = Field(None, description="Output folder (required to copy)")
    buffer_m: float = Field(DEFAULT_BUFFER_M, ge=MIN_BUFFER_M)
    front_overlap: float = Field(DEFAULT_FRONT_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    side_overlap: float = Field(DEFAULT_SIDE_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    copy_photos: bool = Field(True, description="False = report only, copy nothing")

    def to_selection_request(self) -> SelectionRequest:
        return SelectionRequest(
            aoi_shapefile=Path(self.aoi_shapefile), photo_dir=Path(self.photo_dir),
            output_dir=Path(self.output_dir) if self.output_dir else None,
            buffer_m=self.buffer_m, front_overlap=self.front_overlap,
            side_overlap=self.side_overlap, copy_photos=self.copy_photos)


class JobProgress(BaseModel):
    stage: Optional[str] = None      # scan / metadata / geometry / optimize / copy / done
    done: int = 0
    total: int = 0


class JobError(BaseModel):
    code: str                        # INVALID_INPUT / EXPORT_ERROR / INTERNAL_ERROR
    message: str


class SelectionResultV1(BaseModel):
    """Result of a finished job."""

    candidate_photos: int
    selected_photos: int
    removed_photos: int
    reduction_percent: float
    selected: list[PhotoItem] = Field(default_factory=list)
    removed: list[PhotoItem] = Field(default_factory=list)
    original_overlap_below_target: bool = False
    notes: list[str] = Field(default_factory=list)
    output_dir: Optional[str] = None
    copied_photos: int = 0


class JobResponse(BaseModel):
    """POST /api/v1/jobs (201) and GET /api/v1/jobs/{job_id}"""

    job_id: str
    status: JobStatus
    progress: JobProgress = Field(default_factory=JobProgress)
    created_at: datetime
    updated_at: datetime
    request: CreateJobRequest
    error: Optional[JobError] = None


class JobResultResponse(BaseModel):
    """GET /api/v1/jobs/{job_id}/result"""

    job_id: str
    status: JobStatus
    result: SelectionResultV1


class HealthResponse(BaseModel):
    status: str = "ok"
    api_version: str = API_VERSION
    tool_version: str
