"""
Terrain footprints — FootprintMethod.TERRAIN_RASTER (Part 16, Experimental).

Starts from a planar footprint (Height Strategy v2 already applied: pose, lens, LRF gate) and
replaces its ground plane by the terrain height field:

* camera position (x, y) from GPS, Z from ``TerrainVerticalAlignment`` (terrain datum)
* ``boundary_samples`` rays along the recorded image boundary (undistorted with the lens
  model) + the principal ray, first hit on the height field (Part 18)
* failed boundary rays are dropped when their share ≤ ``max_failed_boundary_ratio``
  (Part 21); otherwise the photo has no terrain footprint (its issue code is kept)

Limitations (Part 19): the polygon is the ring of boundary hits, so ground hidden behind a
ridge inside the ring is still counted; a height field has no overhangs; DSM surfaces include
vegetation / buildings.
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from shapely.geometry import Polygon

from ..camera.intrinsics import resolve_intrinsics
from ..camera.models import CameraIntrinsics, LensMode
from ..footprint.base import FootprintUnavailableError, boundary_pixels
from ..footprint.models import FootprintMethod, FootprintWarning, PhotoFootprint
from ..footprint.pose import GridPose
from ..metadata.models import PhotoMetadata
from .alignment import AlignedHeight, TerrainVerticalAlignment
from .base import TerrainProvider
from .models import TerrainConfig, TerrainIssue, TerrainSurfaceKind

DERIVED_FACTOR = 0.9
TERRAIN_FACTOR_REMOVED = 0.8          # planar TERRAIN_NOT_ACCOUNTED_FOR factor


class TerrainFootprintError(FootprintUnavailableError):
    def __init__(self, message: str, issue: Optional[TerrainIssue] = None,
                 counts: Optional[dict] = None):
        super().__init__(message)
        self.issue = issue
        self.counts = counts or {}


def grid_pose_of(fp: PhotoFootprint) -> GridPose:
    p = fp.provenance
    return GridPose(p["yaw_grid_deg"], p["pitch_deg"], p["roll_deg"], p["grid_offset_deg"])


def camera_rays(pose: GridPose, camera: CameraIntrinsics, pixels: np.ndarray) -> np.ndarray:
    """Ideal pinhole pixels → unit ray directions in grid (east, north, up)."""
    body = np.column_stack([np.full(len(pixels), camera.focal_px),
                            pixels[:, 0] - camera.cx_px,
                            (pixels[:, 1] - camera.cy_px) * camera.focal_px / camera.fy])
    north, east, down = pose.rotation_ned() @ body.T
    d = np.column_stack([east, north, -down])
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def project_terrain(planar: PhotoFootprint, camera: CameraIntrinsics, height: AlignedHeight,
                    provider: TerrainProvider, config: TerrainConfig) -> PhotoFootprint:
    pose = grid_pose_of(planar)
    per_edge = max(2, config.boundary_samples // 4)
    boundary, corner_idx = boundary_pixels(camera, edge_samples=per_edge)
    ideal = camera.distortion.undistort_pixels(boundary) if camera.distortion else boundary
    pixels = np.vstack([ideal, [[camera.cx_px, camera.cy_px]]])
    dirs = camera_rays(pose, camera, pixels)
    x0, y0 = planar.camera_xy
    origins = np.tile([x0, y0, height.camera_z], (len(dirs), 1))
    max_range = config.max_range_factor * max(planar.height_m, 1.0)
    res = provider.intersect_rays(origins, dirs, max_range=max_range)
    ring_ok = res.valid[:-1]
    n = len(ring_ok)
    failed = int(n - ring_ok.sum())
    counts = Counter(i.value for i in res.issues[:-1] if i is not None)
    if failed / n > config.max_failed_boundary_ratio + 1e-12 or ring_ok.sum() < 3:
        issue = TerrainIssue(counts.most_common(1)[0][0]) if counts else None
        raise TerrainFootprintError(
            f"{failed}/{n} boundary rays failed ({dict(counts)})", issue, dict(counts))
    pts = res.xyz[:-1][ring_ok]
    poly = Polygon(pts[:, :2])
    warnings = [w for w in planar.warnings if w is not FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR]
    if not poly.is_valid:
        poly = poly.buffer(0)
        warnings.append(FootprintWarning.TERRAIN_FOOTPRINT_REPAIRED)
        if poly.geom_type == "MultiPolygon":
            poly = max(poly.geoms, key=lambda g: g.area)
    if poly.is_empty or poly.area <= 0:
        raise TerrainFootprintError("degenerate terrain footprint",
                                    TerrainIssue.TERRAIN_NO_INTERSECTION)
    if failed:
        warnings.append(FootprintWarning.TERRAIN_PARTIAL_BOUNDARY)
    src = provider.source
    derived = (not src.independent) or src.kind is TerrainSurfaceKind.DERIVED
    if derived:
        warnings.append(FootprintWarning.TERRAIN_SURFACE_DERIVED)
    if height.method == "FLIGHT_OFFSET":
        warnings.append(FootprintWarning.TERRAIN_HEIGHT_FLIGHT_OFFSET)
    conf = planar.confidence
    if FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR in planar.warnings:
        conf /= TERRAIN_FACTOR_REMOVED
    conf *= (DERIVED_FACTOR if derived else 1.0) * (1 - failed / n)
    principal = (tuple(res.xyz[-1, :2]) if res.valid[-1] else None)
    corners = []
    for i in corner_idx:
        corners.append(tuple(res.xyz[i, :2]) if res.valid[i] else (float("nan"),) * 2)
    hz = res.xyz[:-1][ring_ok][:, 2]
    prov = dict(planar.provenance)
    prov.update({
        "terrain_provider": provider.name, "terrain_method": res.method,
        "terrain_source_kind": src.kind.value, "terrain_independent": not derived,
        "terrain_vertical_datum": provider.vertical_datum.value,
        "vertical_alignment": height.method, "camera_z_terrain": round(height.camera_z, 3),
        "camera_offset_m": None if height.offset_m is None else round(height.offset_m, 3),
        "boundary_rays": n, "boundary_failed": failed, "terrain_issues": dict(counts),
        "ground_z_min": round(float(hz.min()), 3), "ground_z_max": round(float(hz.max()), 3),
        "planar_method": planar.method.value,
    })
    return planar.model_copy(update={
        "geometry": poly, "method": FootprintMethod.TERRAIN_RASTER,
        "confidence": round(min(max(conf, 0.0), 1.0), 3), "warnings": warnings,
        "principal_ground_xy": principal, "corner_xy": corners,
        "projector": f"TerrainRaster[{provider.name}]", "provenance": prov})


@dataclass
class TerrainBatchResult:
    footprints: dict[str, PhotoFootprint] = field(default_factory=dict)
    unavailable: dict[str, str] = field(default_factory=dict)
    issues: dict[str, Optional[str]] = field(default_factory=dict)
    issue_counts: Counter = field(default_factory=Counter)
    rays: int = 0
    seconds: float = 0.0


def project_terrain_all(planar: dict[str, PhotoFootprint], metadata: dict[str, PhotoMetadata],
                        provider: TerrainProvider, alignment: TerrainVerticalAlignment,
                        config: TerrainConfig, lens_mode: LensMode = LensMode.AUTO
                        ) -> TerrainBatchResult:
    """Terrain footprints for every planar footprint (photos without one stay without)."""
    out = TerrainBatchResult()
    t0 = time.perf_counter()
    for pid in sorted(planar):
        fp = planar[pid]
        h = alignment.heights.get(pid)
        if h is None:
            out.unavailable[pid] = "no aligned camera height"
            out.issues[pid] = None
            continue
        cam = resolve_intrinsics(metadata[pid], lens_mode=lens_mode)
        try:
            out.footprints[pid] = project_terrain(fp, cam, h, provider, config)
            out.rays += config.boundary_samples + 1
        except TerrainFootprintError as exc:
            out.rays += config.boundary_samples + 1
            out.unavailable[pid] = f"TERRAIN: {exc}"
            out.issues[pid] = exc.issue.value if exc.issue else None
            out.issue_counts[exc.issue.value if exc.issue else "OTHER"] += 1
    out.seconds = time.perf_counter() - t0
    return out
