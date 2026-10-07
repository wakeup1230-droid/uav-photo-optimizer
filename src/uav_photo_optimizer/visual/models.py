"""
Visual connectivity validation models (Phase 8A — Experimental).

Geometric overlap PASS is NOT image-matching PASS. A pair is visually connected only when
feature matches survive a robust epipolar model (Fundamental / Essential matrix — never
Homography as the PASS model) with enough inliers, inlier ratio AND spatial spread.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field


class VisualStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    UNRESOLVED = "VISUAL_UNRESOLVED"     # unreadable / too few features / unstable RANSAC


class EdgeKind(str, Enum):
    NEW_EDGE = "NEW_EDGE"                              # created by removing photos in between
    LOW_MARGIN_EDGE = "LOW_MARGIN_EDGE"                # unchanged, geometric margin < 5 pts
    OBLIQUE_EDGE = "OBLIQUE_EDGE"                      # unchanged oblique adjacency
    BASELINE_DEFECT_NEIGHBOR = "BASELINE_DEFECT_NEIGHBOR"
    BASELINE_EDGE = "BASELINE_EDGE"                    # other unchanged adjacency


class VisualThresholds(BaseModel):
    """
    PASS needs ALL of these. Calibrated on 53 real pairs labelled GOOD / MARGINAL / BAD
    (internal research notes): GOOD ≥ 190 inliers / ratio ≥ 0.82 / ≥ 7 cells; BAD ≤ 18 /
    ≤ 0.53 / ≤ 6. These defaults pass 18/18 GOOD and fail 5/5 MARGINAL and 30/30 BAD.
    """

    model_config = ConfigDict(extra="forbid")

    min_inliers: int = Field(100, ge=1)                # epipolar (E when available, else F)
    min_inlier_ratio: float = Field(0.75, ge=0, le=1)  # inliers / ratio-test matches
    min_grid_cells: int = Field(7, ge=1)               # of 16 (4 × 4) cells with inliers, both images
    min_keypoints: int = Field(200, ge=1)              # below → VISUAL_UNRESOLVED
    min_ratio_matches: int = Field(15, ge=1)           # below with enough keypoints → FAIL


class VisualValidationConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    detector: str = "SIFT"                  # SIFT / AKAZE / ORB
    matcher: str = "FLANN"                  # FLANN / BF
    max_long_edge: int = Field(1024, ge=256)        # benchmark: internal research notes
    ratio_test: float = Field(0.75, gt=0, lt=1)
    max_features: int = Field(8000, ge=100)
    ransac_threshold_px: float = Field(1.5, gt=0)   # at working (resized) resolution
    ransac_confidence: float = Field(0.999, gt=0, lt=1)
    lens_handling: str = "UNDISTORT_KEYPOINTS"      # see internal research notes
    grid: int = Field(4, ge=2)
    thresholds: VisualThresholds = Field(default_factory=VisualThresholds)
    validate_unchanged_edges: bool = True           # LOW_MARGIN / OBLIQUE / DEFECT neighbours


class VisualMatchResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_a: str
    photo_b: str
    detector: str
    matcher: str
    image_long_edge: Optional[int] = None
    keypoints_a: int = 0
    keypoints_b: int = 0
    raw_matches: int = 0
    ratio_test_matches: int = 0
    fundamental_inliers: int = 0
    fundamental_inlier_ratio: float = 0.0
    essential_inliers: Optional[int] = None
    essential_inlier_ratio: Optional[float] = None
    homography_inliers: Optional[int] = None          # diagnostic only, never PASS model
    homography_inlier_ratio: Optional[float] = None
    spatial_coverage_a: float = 0.0                   # occupied grid cells / all cells
    spatial_coverage_b: float = 0.0
    grid_cells_a: int = 0
    grid_cells_b: int = 0
    hull_area_ratio_a: float = 0.0                    # inlier convex hull / image area
    hull_area_ratio_b: float = 0.0
    status: VisualStatus
    confidence: float = Field(0.0, ge=0, le=1)
    warnings: list[str] = Field(default_factory=list)
    processing_time_ms: float = 0.0


class VisualEdge(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_a: str
    photo_b: str
    strip_id: Optional[str] = None
    view_group: str
    kind: EdgeKind
    removed_between: list[str] = Field(default_factory=list)
    geometric_overlap_pct: Optional[float] = None
    result: Optional[VisualMatchResult] = None


class VisualValidationReport(BaseModel):
    model_config = ConfigDict(extra="forbid")

    config: VisualValidationConfig
    edges: list[VisualEdge] = Field(default_factory=list)
    new_edges: int = 0
    new_edges_pass: int = 0
    new_edges_fail: int = 0
    new_edges_unresolved: int = 0
    visual_restored: list[str] = Field(default_factory=list)
    remaining_new_failed_edges: int = 0          # must be 0 for VALIDATED
    baseline_visual_defects: int = 0             # unchanged edges failing (recorded only)
    iterations: int = 0
    images_processed: int = 0
    processing_time_s: float = 0.0
    warnings: list[str] = Field(default_factory=list)
