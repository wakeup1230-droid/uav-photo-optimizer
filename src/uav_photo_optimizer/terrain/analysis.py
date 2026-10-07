"""
Terrain analysis orchestration (Experimental, DRY RUN): preflight → vertical alignment →
TERRAIN_RASTER footprints → overlap on terrain footprints.

The planar geometry is never modified; a separate ``GeometryAnalysis`` is returned that the
optimizer can be run on for comparison. The optimizer default stays planar.
Photos whose terrain footprint fails keep their planar footprint with height_confidence 0
(TERRAIN_FALLBACK_PLANAR): they stay candidates, are PROTECTED by the optimizer and are never
used as overlap evidence (conservative).
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field
from shapely.geometry.base import BaseGeometry

from ..camera.models import LensMode
from ..core.result import GeometryAnalysis
from ..footprint.models import FootprintWarning
from ..metadata.models import PhotoMetadata
from . import make_provider
from .alignment import VerticalTransform, align_vertical
from .footprint import project_terrain_all
from .models import TerrainConfig, TerrainDiagnostics, TerrainStatus
from .preflight import terrain_preflight


class TerrainAnalysis(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    status: TerrainStatus
    diagnostics: TerrainDiagnostics
    geometry: Optional[GeometryAnalysis] = None
    terrain_footprints: int = 0
    terrain_failed: dict[str, Optional[str]] = Field(default_factory=dict)   # pid → issue
    issue_counts: dict[str, int] = Field(default_factory=dict)
    rays: int = 0
    seconds: float = 0.0
    provider: Optional[str] = None


def analyse_terrain(planar: GeometryAnalysis, metadata: dict[str, PhotoMetadata],
                    config: TerrainConfig, aoi: Optional[BaseGeometry] = None,
                    overlap_config=None, transform: Optional[VerticalTransform] = None,
                    lens_mode: LensMode = LensMode.AUTO) -> TerrainAnalysis:
    from ..overlap.engine import OverlapConfig, OverlapEngine, overlap_statistics

    started = datetime.now()
    fps = [f.geometry for f in planar.footprints.values()]
    if aoi is not None:                          # the rays that matter: AOI candidates
        import shapely
        shapely.prepare(aoi)
        fps = [f for f in fps if shapely.intersects(aoi, f)]
    diag = terrain_preflight(config.source, aoi=aoi, aoi_crs=planar.crs, footprints=fps,
                             min_aoi_coverage=config.min_aoi_coverage,
                             min_ray_coverage=config.min_ray_coverage)
    if diag.crs and planar.crs:
        from pyproj import CRS
        if not CRS(diag.crs).equals(CRS(planar.crs)):
            diag.reasons.append(f"terrain CRS {diag.crs} ≠ footprint CRS {planar.crs}")
            diag.status = TerrainStatus.NOT_READY
    if not diag.ready:
        return TerrainAnalysis(status=TerrainStatus.NOT_READY, diagnostics=diag)

    provider = make_provider(config)
    try:
        flights = {pid: t.flight_id for pid, t in planar.flights.photos.items()} \
            if planar.flights else {}
        al = align_vertical(provider, config, metadata, planar.footprints, flights,
                            transform=transform, target_crs=planar.crs)
        diag.alignment = al.summary()
        if al.status is not TerrainStatus.READY:
            diag.status = TerrainStatus.NOT_READY
            diag.reasons += al.reasons
            return TerrainAnalysis(status=TerrainStatus.NOT_READY, diagnostics=diag,
                                   provider=provider.name)
        batch = project_terrain_all(planar.footprints, metadata, provider, al, config,
                                    lens_mode)
    finally:
        provider.close()

    # terrain failure → keep the planar footprint for candidacy, but with height_confidence 0:
    # the optimizer then protects the photo and never uses it as overlap evidence
    footprints = dict(batch.footprints)
    for pid in sorted(set(planar.footprints) - set(batch.footprints)):
        fp = planar.footprints[pid]
        footprints[pid] = fp.model_copy(update={
            "height_confidence": 0.0, "confidence": 0.0,
            "warnings": fp.warnings + [FootprintWarning.TERRAIN_FALLBACK_PLANAR],
            "provenance": {**fp.provenance, "terrain_issue": batch.issues.get(pid),
                           "terrain_unavailable": batch.unavailable.get(pid)}})

    ocfg = overlap_config or OverlapConfig()
    engine = OverlapEngine(ocfg)
    times = {m.photo_id: m.capture_time_utc or m.capture_time for m in metadata.values()}
    report = engine.compute(footprints, planar.flights, times)
    stats = overlap_statistics(report, planar.flights)
    unavailable = dict(planar.footprint_unavailable)
    warnings = ["Experimental: TERRAIN_RASTER footprints (dry run).",
                f"terrain source: {config.source.kind.value}"
                f"{'' if config.source.independent else ' (NOT independent)'}",
                f"vertical alignment: {al.mode.value}"]
    if batch.unavailable:
        warnings.append(f"{len(batch.unavailable)} photos without terrain footprint: planar "
                        f"footprint kept with height_confidence 0 (TERRAIN_FALLBACK_PLANAR → "
                        f"protected, never overlap evidence)")
    geom = GeometryAnalysis(crs=planar.crs, photos_scanned=planar.photos_scanned,
                            footprints=footprints, footprint_unavailable=unavailable,
                            height_resolutions=planar.height_resolutions,
                            flights=planar.flights, overlap=report, statistics=stats,
                            warnings=warnings, started_at=started, finished_at=datetime.now())
    return TerrainAnalysis(status=TerrainStatus.READY, diagnostics=diag, geometry=geom,
                           terrain_footprints=len(batch.footprints),
                           terrain_failed=batch.issues, issue_counts=dict(batch.issue_counts),
                           rays=batch.rays, seconds=round(batch.seconds, 2),
                           provider=provider.name)
