"""
Terrain models (Phase 7, Experimental).

All terrain work is offline: a local GeoTIFF height field in a *projected metric* CRS.
No network DEM, no geoid grid in Core. Vertical datums are explicit; an unknown datum can
only be bridged by LRF anchoring (``VerticalAlignmentMode.LRF_ANCHORED``) or by an explicit
user statement (``ASSUME_SAME_DATUM``) — never automatically.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Any, Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field


class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid")


class TerrainIssue(str, Enum):
    """Per-ray failure codes (Part 20)."""

    TERRAIN_OUTSIDE = "TERRAIN_OUTSIDE"                  # ray left the raster before a hit
    TERRAIN_NODATA = "TERRAIN_NODATA"                    # touched nodata before a hit
    TERRAIN_NO_INTERSECTION = "TERRAIN_NO_INTERSECTION"  # no hit within the search range
    TERRAIN_NONCONVERGENT = "TERRAIN_NONCONVERGENT"      # refinement did not converge


class TerrainSurfaceKind(str, Enum):
    DEM = "DEM"            # bare earth
    DSM = "DSM"            # surface incl. vegetation / buildings
    DERIVED = "DERIVED"    # derived from the same photos / LRF (not independent)
    UNKNOWN = "UNKNOWN"


class VerticalDatumKind(str, Enum):
    ORTHOMETRIC = "ORTHOMETRIC"      # e.g. EPSG:8904 TWVD 2001 height
    ELLIPSOIDAL = "ELLIPSOIDAL"
    LOCAL = "LOCAL"                  # consistent but unreferenced (e.g. DJI AbsoluteAltitude)
    UNKNOWN = "UNKNOWN"


class TerrainSource(_Model):
    """Where the height field comes from (Part 33). ``path`` is never committed."""

    path: Path
    kind: TerrainSurfaceKind = TerrainSurfaceKind.UNKNOWN
    vertical_datum: VerticalDatumKind = VerticalDatumKind.UNKNOWN
    vertical_crs: Optional[str] = None          # e.g. "EPSG:8904"; None if not referenced
    vertical_unit: str = "metre"
    description: str = ""
    independent: bool = True                    # False for photo / LRF-derived surfaces


class VerticalAlignmentMode(str, Enum):
    """Part 14. AUTO = LRF_ANCHORED only; ASSUME_SAME_DATUM is never chosen automatically."""

    AUTO = "AUTO"
    LRF_ANCHORED = "LRF_ANCHORED"
    EXPLICIT_VERTICAL_TRANSFORM = "EXPLICIT_VERTICAL_TRANSFORM"
    ASSUME_SAME_DATUM = "ASSUME_SAME_DATUM"


class TerrainEngine(str, Enum):
    REFERENCE = "REFERENCE"      # ReferenceRasterRaySolver (rasterio + NumPy)
    WEITSICHT = "WEITSICHT"      # WeitsichtRasterTerrainProvider (optional, pinned)


class TerrainConfig(_Model):
    """Part 33. Terrain footprint configuration (Experimental; optimizer default unchanged)."""

    source: TerrainSource
    engine: TerrainEngine = TerrainEngine.REFERENCE
    alignment: VerticalAlignmentMode = VerticalAlignmentMode.AUTO
    explicit_offset_m: Optional[float] = None       # EXPLICIT: Z_dem = Z_photo + offset
    assume_same_datum_confirmed: bool = False       # must be set by the user explicitly
    boundary_samples: int = Field(64, ge=8, le=1024)
    max_failed_boundary_ratio: float = Field(0.0, ge=0, le=0.5)
    max_range_factor: float = Field(20.0, gt=1)
    preload: bool = True
    min_lrf_anchors: int = Field(20, ge=1)
    max_anchor_offset_spread_m: float = Field(3.0, gt=0)   # robust spread of camera offsets
    step_fraction: float = Field(0.5, gt=0, le=1)           # march step = fraction × pixel
    min_aoi_coverage: float = Field(0.95, ge=0, le=1)       # preflight: AOI on valid cells
    min_ray_coverage: float = Field(0.95, ge=0, le=1)       # preflight: footprints in raster


class TerrainStatus(str, Enum):
    READY = "READY"
    NOT_READY = "NOT_READY"


class TerrainCheck(_Model):
    name: str
    ok: bool
    detail: str = ""


class TerrainDiagnostics(_Model):
    """Part 33. Preflight + alignment diagnostics."""

    status: TerrainStatus
    checks: list[TerrainCheck] = Field(default_factory=list)
    crs: Optional[str] = None
    vertical_datum: VerticalDatumKind = VerticalDatumKind.UNKNOWN
    resolution: Optional[tuple[float, float]] = None
    nodata: Optional[float] = None
    bounds: Optional[tuple[float, float, float, float]] = None
    nodata_fraction: Optional[float] = None
    aoi_coverage: Optional[float] = None
    alignment: Optional[dict[str, Any]] = None
    reasons: list[str] = Field(default_factory=list)

    @property
    def ready(self) -> bool:
        return self.status is TerrainStatus.READY


class TerrainIntersectionResult(BaseModel):
    """Part 4. Vectorised result of ``TerrainProvider.intersect_rays`` (N rays)."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    xyz: np.ndarray                       # (N, 3), NaN where invalid
    valid: np.ndarray                     # (N,) bool
    provider: str
    method: str
    iterations: np.ndarray                # (N,) int
    inside_bounds: np.ndarray             # (N,) bool: ray start XY inside the raster
    nodata_hit: np.ndarray                # (N,) bool
    issues: list[Optional[TerrainIssue]]  # per ray; None when valid
    vertical_datum: VerticalDatumKind = VerticalDatumKind.UNKNOWN
    confidence: float = 1.0
    warnings: list[str] = Field(default_factory=list)

    @property
    def n_valid(self) -> int:
        return int(np.count_nonzero(self.valid))

    def issue_counts(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for i in self.issues:
            if i is not None:
                out[i.value] = out.get(i.value, 0) + 1
        return out
