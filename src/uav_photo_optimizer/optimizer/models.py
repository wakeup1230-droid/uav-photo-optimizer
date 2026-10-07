"""
Photo Selection Optimizer contract (Phase 6A — Experimental, dry run only).

The optimizer never copies, moves or deletes anything: it produces a ``SelectionPlan``
(JSON-serialisable; the future REST API response core). Principle: remove only provably
redundant photos; existing (baseline) defects may remain but must never get worse.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, ConfigDict, Field

from ..core.config import (DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP, MAX_OVERLAP, MIN_OVERLAP,
                           OptimizerConfig)

__all__ = ["OptimizerConfig", "SelectionOptimizerConfig", "DecisionStatus", "SelectionReason",
           "DefectType", "DefectSeverity", "BaselineDefect", "PhotoDecision", "CoverageSummary",
           "SafetyReport", "StripEvaluation", "PlanStatus", "SelectionPlan", "MIN_OVERLAP",
           "MAX_OVERLAP"]


class SelectionOptimizerConfig(BaseModel):
    """Parameters of the Phase 6A optimizer (validation shared with Core / API)."""

    model_config = ConfigDict(extra="forbid")

    front_target_pct: float = Field(DEFAULT_FRONT_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    side_target_pct: float = Field(DEFAULT_SIDE_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    boundary_guard_m: float = Field(20.0, ge=0)          # band inside the coverage-area border
    boundary_min_multiplicity: int = Field(3, ge=1)      # keep ≥ min(baseline, k) views there
    boundary_sample_spacing_m: float = Field(10.0, gt=0)
    defect_protection_neighbours: int = Field(1, ge=0)   # extra photos protected each side
    coverage_tolerance_m2: float = Field(1.0, ge=0)      # numerical tolerance per group
    low_confidence_height: float = Field(0.6, ge=0, le=1)   # height_confidence below → protect
    optimize_nadir_along: bool = True
    evaluate_nadir_strips: bool = True                   # whole-strip removal (cross-track)
    optimize_oblique_along: bool = True
    max_restore_iterations: int = Field(100, ge=1)


class DecisionStatus(str, Enum):
    KEEP = "KEEP"              # considered, required
    REMOVE = "REMOVE"          # provably redundant (dry run: nothing is deleted)
    PROTECTED = "PROTECTED"    # never considered for removal


class SelectionReason(str, Enum):
    KEEP_REQUIRED_FRONT_OVERLAP = "KEEP_REQUIRED_FRONT_OVERLAP"
    KEEP_REQUIRED_ALONG_TRACK_OVERLAP = "KEEP_REQUIRED_ALONG_TRACK_OVERLAP"   # oblique
    KEEP_REQUIRED_SIDE_OVERLAP = "KEEP_REQUIRED_SIDE_OVERLAP"
    KEEP_AOI_BOUNDARY = "KEEP_AOI_BOUNDARY"
    KEEP_STRIP_ENDPOINT = "KEEP_STRIP_ENDPOINT"
    KEEP_CAPTURE_DIRECTION = "KEEP_CAPTURE_DIRECTION"
    KEEP_NOT_OPTIMIZED = "KEEP_NOT_OPTIMIZED"            # group / step disabled
    PROTECTED_BASELINE_DEFECT = "PROTECTED_BASELINE_DEFECT"
    PROTECTED_LOW_CONFIDENCE_HEIGHT = "PROTECTED_LOW_CONFIDENCE_HEIGHT"
    PROTECTED_HEIGHT_UNRESOLVED = "PROTECTED_HEIGHT_UNRESOLVED"
    PROTECTED_TURN = "PROTECTED_TURN"
    REMOVE_REDUNDANT_ALONG_TRACK = "REMOVE_REDUNDANT_ALONG_TRACK"            # nadir
    REMOVE_REDUNDANT_OBLIQUE_ALONG_TRACK = "REMOVE_REDUNDANT_OBLIQUE_ALONG_TRACK"
    REMOVE_REDUNDANT_CROSS_TRACK = "REMOVE_REDUNDANT_CROSS_TRACK"            # whole strip
    RESTORED_BY_VALIDATION = "RESTORED_BY_VALIDATION"
    RESTORED_BY_VISUAL_VALIDATION = "RESTORED_BY_VISUAL_VALIDATION"


class DefectType(str, Enum):
    FRONT_GAP = "FRONT_GAP"                      # nadir, along-track < front target
    SIDE_GAP = "SIDE_GAP"                        # nadir, cross-track < side target
    ALONG_TRACK_GAP = "ALONG_TRACK_GAP"          # oblique (same look group)
    CROSS_TRACK_GAP = "CROSS_TRACK_GAP"          # oblique (same look group)
    AOI_COVERAGE_GAP = "AOI_COVERAGE_GAP"        # coverage area not seen by any candidate
    LOW_CONFIDENCE_GEOMETRY = "LOW_CONFIDENCE_GEOMETRY"


class DefectSeverity(str, Enum):
    LOW = "LOW"          # below target, ≥ target − 5 points
    MEDIUM = "MEDIUM"    # below target − 5 points, ≥ 65 %
    HIGH = "HIGH"        # below the absolute minimum (65 %) / coverage hole


class BaselineDefect(BaseModel):
    model_config = ConfigDict(extra="forbid")

    defect_id: str
    type: DefectType
    flight_id: Optional[str] = None
    strip_id: Optional[str] = None
    strip_id_b: Optional[str] = None
    view_group: Optional[str] = None
    affected_photos: list[str] = Field(default_factory=list)
    affected_geometry: Optional[dict] = None     # GeoJSON (coverage gaps)
    baseline_metric: Optional[float] = None      # % (overlap) or m² (coverage gap)
    target_metric: Optional[float] = None
    severity: DefectSeverity
    detail: str = ""


class PhotoDecision(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_id: str
    status: DecisionStatus
    reason: SelectionReason
    capture_type: str
    view_group: str
    flight_id: Optional[str] = None
    strip_id: Optional[str] = None
    detail: dict[str, Any] = Field(default_factory=dict)


class CoverageSummary(BaseModel):
    model_config = ConfigDict(extra="forbid")

    area_m2: float                                   # coverage area (AOI + buffer)
    covered_m2: float                                # all groups
    covered_ratio: float
    by_group_m2: dict[str, float] = Field(default_factory=dict)


class SafetyReport(BaseModel):
    """Must all be 0 for a VALID plan."""

    model_config = ConfigDict(extra="forbid")

    new_coverage_holes: int = 0
    new_coverage_hole_m2: float = 0.0
    new_front_defects: int = 0
    new_side_defects: int = 0
    new_along_track_defects: int = 0        # oblique
    new_cross_track_defects: int = 0        # oblique
    baseline_defects_worsened: int = 0
    boundary_multiplicity_violations: int = 0
    details: list[str] = Field(default_factory=list)

    @property
    def total(self) -> int:
        return (self.new_coverage_holes + self.new_front_defects + self.new_side_defects
                + self.new_along_track_defects + self.new_cross_track_defects
                + self.baseline_defects_worsened + self.boundary_multiplicity_violations)


class StripEvaluation(BaseModel):
    model_config = ConfigDict(extra="forbid")

    strip_id: str
    left_strip: Optional[str] = None
    right_strip: Optional[str] = None
    neighbours_side_overlap_min_pct: Optional[float] = None
    removable: bool = False
    removed: bool = False
    reason: str = ""


class PlanStatus(str, Enum):
    VALID = "VALID"            # geometric safety passed → exportable (official pipeline)
    VALIDATED = "VALIDATED"    # geometric + optional visual guard passed → exportable
    INVALID = "INVALID"


class SelectionPlan(BaseModel):
    model_config = ConfigDict(extra="forbid")

    status: PlanStatus
    dry_run: bool = True
    config: SelectionOptimizerConfig
    input_photo_count: int
    candidate_photo_count: int
    keep_count: int
    remove_count: int
    protected_count: int
    reduction_percent: float                 # remove / candidates × 100
    counts_by_capture: dict[str, dict[str, int]] = Field(default_factory=dict)
    counts_by_reason: dict[str, int] = Field(default_factory=dict)
    restored_count: int = 0                  # geometric restores
    visual_restore_count: int = 0
    geometry_valid: Optional[bool] = None
    visual_valid: Optional[bool] = None      # None = visual guard not run
    exportable: bool = False                 # VALID (geometric) or VALIDATED; never INVALID
    decisions: list[PhotoDecision] = Field(default_factory=list)
    baseline_defects: list[BaselineDefect] = Field(default_factory=list)
    coverage_before: Optional[CoverageSummary] = None
    coverage_after: Optional[CoverageSummary] = None
    strip_evaluations: list[StripEvaluation] = Field(default_factory=list)
    safety: SafetyReport = Field(default_factory=SafetyReport)
    constraints_passed: bool = False
    visual: Optional[Any] = None             # VisualValidationReport (when the guard ran)
    warnings: list[str] = Field(default_factory=list)
    created_at: Optional[datetime] = None
