"""
Overlap models (Phase 5). Calculation only — nothing here removes photos.

Terminology (docs/OVERLAP_METHOD.md):

    NADIR    FRONT_OVERLAP                   consecutive nadir photos, same strip, along-track
             SIDE_OVERLAP                    nadir photos of neighbouring strips, cross-track
    OBLIQUE  ALONG_TRACK_GEOMETRIC_OVERLAP   consecutive same-look-direction obliques, same strip
             CROSS_TRACK_GEOMETRIC_OVERLAP   same-look-direction obliques, neighbouring strips

Directional metrics come from projecting the Estimated Ground Footprints onto the strip
coordinate frame (along-track axis = strip travel heading, cross-track axis = its normal).
``SharedCoverage`` (polygon intersection) is reported for every pair but is NOT a substitute
for front / side overlap.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field


class OverlapKind(str, Enum):
    FRONT_OVERLAP = "FRONT_OVERLAP"
    SIDE_OVERLAP = "SIDE_OVERLAP"
    ALONG_TRACK_GEOMETRIC_OVERLAP = "ALONG_TRACK_GEOMETRIC_OVERLAP"
    CROSS_TRACK_GEOMETRIC_OVERLAP = "CROSS_TRACK_GEOMETRIC_OVERLAP"


class OverlapMethod(str, Enum):
    NADIR_GEOMETRIC_PLANAR = "NADIR_GEOMETRIC_PLANAR"
    OBLIQUE_GEOMETRIC_PLANAR = "OBLIQUE_GEOMETRIC_PLANAR"
    IMAGE_MATCHING = "IMAGE_MATCHING"          # Planned (Phase 8 visual validation)


class TerrainMode(str, Enum):
    LOCAL_HORIZONTAL_PLANE = "LOCAL_HORIZONTAL_PLANE"   # per-photo planar footprints (Phase 5)
    TERRAIN_PROVIDER = "TERRAIN_PROVIDER"               # Phase 7: TERRAIN_RASTER footprints


class AxisOverlap(BaseModel):
    """1-D overlap of the two footprints projected on one strip axis (metres)."""

    model_config = ConfigDict(extra="forbid")

    axis: str                          # "ALONG" / "CROSS"
    axis_heading: float                # grid bearing of the projection axis
    length_a: float
    length_b: float
    overlap_length: float
    over_min: float                    # overlap / min(length_a, length_b)
    over_a: float                      # overlap / length_a
    over_b: float                      # overlap / length_b
    over_max: float                    # overlap / max(...) = min(over_a, over_b)  (PRIMARY)


class SharedCoverage(BaseModel):
    model_config = ConfigDict(extra="forbid")

    area_a: float
    area_b: float
    intersection_area: float
    over_a: float
    over_b: float
    over_min_area: float
    iou: float


class OverlapResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_a: str
    photo_b: str
    kind: OverlapKind
    value: float                        # primary metric (fraction 0–1) = axis.over_max
    axis: AxisOverlap
    shared: SharedCoverage
    strip_a: Optional[str] = None
    strip_b: Optional[str] = None
    same_strip: bool
    neighbour_rank: int = 1             # 1 = consecutive; 2 = next-but-one (same strip only)
    look_sector: Optional[int] = None   # oblique absolute look direction sector (0–7 × 45°)
    distance_m: float                   # camera-to-camera
    capture_time_delta_s: Optional[float] = None
    geometry_method: OverlapMethod
    terrain_mode: TerrainMode = TerrainMode.LOCAL_HORIZONTAL_PLANE
    height_strategy: str
    confidence: float = Field(ge=0, le=1)
    warnings: list[str] = Field(default_factory=list)
    target_pct: Optional[float] = None
    target_pass: Optional[bool] = None  # evaluation only — never used to remove photos
    valid: bool = True


class StripPair(BaseModel):
    model_config = ConfigDict(extra="forbid")

    strip_a: str
    strip_b: str
    side: str                           # "LEFT" / "RIGHT" of strip_a (travel direction)
    axis_difference_deg: float
    cross_track_distance_m: float
    along_track_overlap_ratio: float    # shared along-track extent / shorter strip
    same_flight: bool


class PhotoAdjacencyGraph(BaseModel):
    """Photos as nodes, OverlapResults as edges. Data only — no traversal / reduction."""

    model_config = ConfigDict(extra="forbid")

    nodes: list[str] = Field(default_factory=list)
    edges: list[OverlapResult] = Field(default_factory=list)

    def neighbours(self, photo_id: str) -> list[OverlapResult]:
        return [e for e in self.edges if photo_id in (e.photo_a, e.photo_b)]

    def degree(self) -> dict[str, int]:
        d = {n: 0 for n in self.nodes}
        for e in self.edges:
            d[e.photo_a] = d.get(e.photo_a, 0) + 1
            d[e.photo_b] = d.get(e.photo_b, 0) + 1
        return d


class OverlapReport(BaseModel):
    model_config = ConfigDict(extra="forbid")

    crs: str
    front_target_pct: float
    side_target_pct: float
    strip_pairs: list[StripPair] = Field(default_factory=list)
    graph: PhotoAdjacencyGraph = Field(default_factory=PhotoAdjacencyGraph)
    warnings: list[str] = Field(default_factory=list)

    def results(self, kind: OverlapKind, neighbour_rank: int = 1) -> list[OverlapResult]:
        return [e for e in self.graph.edges if e.kind is kind and e.neighbour_rank == neighbour_rank]
