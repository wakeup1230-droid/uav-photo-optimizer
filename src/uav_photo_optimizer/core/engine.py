"""
Core Engine — the single entry point for every front end.

    from uav_photo_optimizer import UAVPhotoOptimizer, RunConfig

    result = UAVPhotoOptimizer(RunConfig.create(buffer_m=100)).run()

The engine only READS input (shapefile, photo metadata) and returns a SelectionResult.
It never moves / renames / modifies input and never copies files; see
``uav_photo_optimizer.export`` for the optional copy step.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from ..aoi.buffer import AOIBuffer, build_buffer
from ..aoi.crs import crs_label
from ..aoi.loader import find_shapefile, load_shapefile
from ..aoi.selection import select_by_footprint, select_by_gps_point
from ..footprint.base import FootprintProjector, ProjectionContext
from ..metadata.exiftool import ExifToolAdapter, photo_id_for
from ..metadata.models import PhotoMetadata
from ..metadata.scanner import find_photos
from ..paths import ProjectPaths
from .config import CandidateSelectionMode, FootprintProjectorKind, RunConfig
from .exceptions import UAVPhotoOptimizerError
from .result import AOISummary, GeometryAnalysis, PhotoReason, PhotoRecord, SelectionResult

ProgressCallback = Callable[[str, int, int], None]   # (stage, done, total)

PHASE1_OPTIMIZER_WARNING = (
    "Overlap optimization not implemented yet (Planned: Phase 6); "
    "selected_photos = all candidate_photos.")


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str = ""


def _record(meta: PhotoMetadata, reason: PhotoReason, message: Optional[str] = None,
            footprint=None) -> PhotoRecord:
    return PhotoRecord(photo_id=meta.photo_id, filename=meta.filename, path=meta.path,
                       reason=reason, message=message, metadata=meta, footprint=footprint)


def make_projector(kind: FootprintProjectorKind) -> FootprintProjector:
    if kind is FootprintProjectorKind.PLANAR:
        from ..footprint.planar import PlanarProjector
        return PlanarProjector()
    from ..footprint.cameratransform_adapter import CameraTransformProjector
    return CameraTransformProjector()


class UAVPhotoOptimizer:
    def __init__(self, config: Optional[RunConfig] = None):
        self.config = config or RunConfig()
        self.paths = ProjectPaths.discover(self.config.base_dir)
        self.exiftool = ExifToolAdapter(self.config.exiftool_path, paths=self.paths)

    # -- resolved inputs ------------------------------------------------------

    @property
    def photo_dir(self) -> Path:
        return Path(self.config.photo_dir) if self.config.photo_dir else self.paths.photo_dir

    @property
    def shp_dir(self) -> Path:
        return Path(self.config.shp_dir) if self.config.shp_dir else self.paths.shp_dir

    def resolve_shapefile(self) -> Path:
        if self.config.aoi.shapefile:
            return Path(self.config.aoi.shapefile)
        return find_shapefile(self.shp_dir)

    def build_aoi(self, warnings: Optional[list[str]] = None) -> tuple[Path, AOIBuffer]:
        shp = self.resolve_shapefile()
        gdf = load_shapefile(shp, warnings)
        return shp, build_buffer(gdf, self.config.aoi.buffer_m)

    # -- preflight ------------------------------------------------------------

    def preflight(self) -> list[CheckResult]:
        """Non-raising readiness checks (read only)."""
        checks: list[CheckResult] = []

        def check(name, func):
            try:
                checks.append(CheckResult(name, True, func() or ""))
                return True
            except UAVPhotoOptimizerError as exc:
                checks.append(CheckResult(name, False, str(exc)))
                return False

        check("Project Root", lambda: str(self.paths.base_dir))
        if check("AOI shapefile", lambda: str(self.resolve_shapefile())):
            def aoi():
                _, buf = self.build_aoi()
                return (f"{crs_label(buf.source_crs)} → buffer {buf.buffer_m:g} m "
                        f"in {crs_label(buf.crs)}")
            check("AOI CRS / buffer", aoi)
        check("Photos", lambda: f"{len(find_photos(self.photo_dir))} JPG in {self.photo_dir}")
        check("ExifTool", lambda: f"{self.exiftool.version} ({self.exiftool.locate()})")
        return checks

    # -- run ------------------------------------------------------------------

    def run(self, progress: Optional[ProgressCallback] = None) -> SelectionResult:
        cfg = self.config
        projector = (make_projector(cfg.footprint_projector)
                     if cfg.selection_mode is CandidateSelectionMode.FOOTPRINT else None)
        started = datetime.now()
        warnings: list[str] = []

        shp, aoi = self.build_aoi(warnings)
        photos = find_photos(self.photo_dir)
        read = self.exiftool.read(
            photos, photo_root=self.photo_dir,
            progress=(lambda d, t: progress("metadata", d, t)) if progress else None)

        with_gps = [m for m in read.photos if m.has_gps]
        no_gps = [_record(m, PhotoReason.NO_GPS) for m in read.photos if not m.has_gps]
        errors = [PhotoRecord(photo_id=photo_id_for(p, self.photo_dir), filename=p.name, path=p,
                              reason=PhotoReason.READ_ERROR, message=msg)
                  for p, msg in read.errors]

        if projector is None:
            inside, outside = select_by_gps_point(with_gps, aoi)
            candidates = [_record(m, PhotoReason.SELECTED) for m in inside]
            rejected = [_record(m, PhotoReason.OUTSIDE_AOI) for m in outside]
        else:
            context = ProjectionContext(target_crs=crs_label(aoi.crs),
                                        height_strategy=cfg.height_strategy,
                                        user_ground_elevation=cfg.user_ground_elevation)
            sel = select_by_footprint(with_gps, aoi, projector, context, cfg.lens_mode)
            candidates = ([_record(m, PhotoReason.SELECTED, footprint=fp)
                           for m, fp in sel.inside]
                          + [_record(m, PhotoReason.SELECTED, message=why)
                             for m, why in sel.unresolved_inside])
            rejected = ([_record(m, PhotoReason.OUTSIDE_AOI, footprint=fp)
                         for m, fp in sel.outside]
                        + [_record(m, PhotoReason.FOOTPRINT_UNAVAILABLE, message=why)
                           for m, why in sel.unavailable])
            warnings.append(
                f"FOOTPRINT mode uses Estimated Ground Footprints ({projector.name}, "
                f"height strategy {cfg.height_strategy.value}, lens {cfg.lens_mode.value}); "
                "terrain is not accounted for.")
            warnings.extend(f"footprint warning {k}: {v} photos"
                            for k, v in sorted(sel.warning_counts.items()))
            if sel.unavailable:
                warnings.append(f"{len(sel.unavailable)} photos without an estimable footprint")
            if sel.unresolved_inside:
                warnings.append(f"{len(sel.unresolved_inside)} HEIGHT_UNRESOLVED photos kept as "
                                "candidates by their GPS point (conservative)")

        # Phase 1: no optimizer yet → every candidate is selected.
        selected = list(candidates)
        warnings.append(PHASE1_OPTIMIZER_WARNING)

        return SelectionResult(
            photos_scanned=len(photos),
            candidate_photos=candidates,
            selected_photos=selected,
            rejected_photos=rejected,
            no_gps=no_gps,
            errors=errors,
            buffer_m=cfg.optimizer.buffer_m,
            front_overlap_target=cfg.optimizer.front_overlap_target,
            side_overlap_target=cfg.optimizer.side_overlap_target,
            selection_mode=cfg.selection_mode,
            coverage_valid=None,
            warnings=warnings,
            aoi=AOISummary(shapefile=shp, source_crs=crs_label(aoi.source_crs),
                           buffer_crs=crs_label(aoi.crs), buffer_m=aoi.buffer_m,
                           buffer_area_m2=float(aoi.geometry.area)),
            started_at=started,
            finished_at=datetime.now(),
        )

    # -- geometry analysis (Phase 3.1 – 5): evaluation only, never removes photos ----------

    def analyse_geometry(self, progress: Optional[ProgressCallback] = None,
                         flight_config=None) -> GeometryAnalysis:
        """
        Estimated Ground Footprints (all photos with GPS) → flights / strips → overlap.

        Uses ``config.height_strategy`` / ``lens_mode`` / ``footprint_projector`` and the
        overlap targets of ``config.optimizer``. Selection (``run``) is not affected.
        """
        photos = find_photos(self.photo_dir)
        read = self.exiftool.read(
            photos, photo_root=self.photo_dir,
            progress=(lambda d, t: progress("metadata", d, t)) if progress else None)
        g = self.geometry_from_metadata(read.photos, len(photos), flight_config)
        if read.errors:
            g.warnings.append(f"{len(read.errors)} metadata read errors")
        return g

    def geometry_from_metadata(self, metadata, photos_scanned: int,
                               flight_config=None) -> GeometryAnalysis:
        """Geometry analysis from already-read metadata (READ ONLY)."""
        from ..flight.strip import FlightConfig, analyse_flights
        from ..footprint.batch import project_all
        from ..overlap.engine import OverlapConfig, OverlapEngine, overlap_statistics

        cfg = self.config
        started = datetime.now()
        _, aoi = self.build_aoi()
        crs = crs_label(aoi.crs)
        meta = [m for m in metadata if m.has_gps]
        projector = make_projector(cfg.footprint_projector)
        context = ProjectionContext(target_crs=crs, height_strategy=cfg.height_strategy,
                                    user_ground_elevation=cfg.user_ground_elevation)
        flights = analyse_flights(meta, crs, flight_config or FlightConfig())
        batch = project_all(meta, flights, projector, context, cfg.lens_mode)
        engine = OverlapEngine(OverlapConfig(
            front_target_pct=cfg.optimizer.front_overlap_target,
            side_target_pct=cfg.optimizer.side_overlap_target))
        times = {m.photo_id: m.capture_time_utc or m.capture_time for m in meta}
        report = engine.compute(batch.footprints, flights, times)
        stats = overlap_statistics(report, flights)
        warnings = ["Evaluation only: overlap targets are checked (PASS/FAIL) but no photo is "
                    "removed here.",
                    "TERRAIN_NOT_ACCOUNTED_FOR: footprints are planar estimates."]
        if batch.unavailable:
            warnings.append(f"{len(batch.unavailable)} photos without footprint "
                            f"({len(batch.unresolved)} HEIGHT_UNRESOLVED)")
        return GeometryAnalysis(crs=crs, photos_scanned=photos_scanned,
                                footprints=batch.footprints,
                                footprint_unavailable=batch.unavailable,
                                height_resolutions=batch.resolutions, flights=flights,
                                overlap=report, statistics=stats, warnings=warnings,
                                started_at=started, finished_at=datetime.now())

    # -- Phase 7 (Experimental): terrain-aware footprints, DRY RUN ------------------------

    def analyse_terrain(self, geometry: GeometryAnalysis, metadata: dict, terrain_config,
                        transform=None):
        """
        TERRAIN_RASTER footprints + overlap from a planar ``geometry`` (unchanged) and a local
        GeoTIFF (``TerrainConfig``) → ``TerrainAnalysis``. NOT_READY when the raster fails
        preflight or the vertical datum cannot be bridged. Nothing is written or copied.
        """
        from ..overlap.engine import OverlapConfig
        from ..terrain.analysis import analyse_terrain

        _, aoi = self.build_aoi()
        ocfg = OverlapConfig(front_target_pct=self.config.optimizer.front_overlap_target,
                             side_target_pct=self.config.optimizer.side_overlap_target)
        return analyse_terrain(geometry, metadata, terrain_config, aoi=aoi.geometry,
                               overlap_config=ocfg, transform=transform,
                               lens_mode=self.config.lens_mode)

    # -- selection plan (Phase 6A, Experimental): DRY RUN ONLY ---------------------------

    def plan_selection(self, selection_config=None, geometry: Optional[GeometryAnalysis] = None,
                       progress: Optional[ProgressCallback] = None):
        """
        Dry-run Photo Selection Optimizer → ``SelectionPlan``. Nothing is copied or deleted.
        Coverage area = AOI + buffer. Targets default to ``config.optimizer``.
        """
        from ..optimizer.models import SelectionOptimizerConfig
        from ..optimizer.planner import plan_selection

        if selection_config is None:
            selection_config = SelectionOptimizerConfig(
                front_target_pct=self.config.optimizer.front_overlap_target,
                side_target_pct=self.config.optimizer.side_overlap_target)
        geometry = geometry or self.analyse_geometry(progress)
        _, aoi = self.build_aoi()
        return plan_selection(geometry, aoi.geometry, selection_config)

    # -- Phase 8A / 6B: visual guard + validated export ------------------------------------

    def validate_selection(self, plan, geometry: GeometryAnalysis, metadata: dict,
                           visual_config=None, validator=None):
        """Visual Safety Guard on a geometric plan → plan with status VALIDATED / INVALID.
        ``metadata``: photo_id → PhotoMetadata (paths for reading images, READ ONLY)."""
        from ..optimizer.validated import validate_plan
        if validator is None:
            from ..visual.opencv_validator import OpenCVVisualValidator
            validator = OpenCVVisualValidator(visual_config)
        _, aoi = self.build_aoi()
        return validate_plan(geometry, aoi.geometry, plan, metadata, validator, visual_config,
                             lens_mode=self.config.lens_mode)

    def export_selection(self, plan, metadata: dict, output_root=None,
                         require_visual: bool = False):
        """COPY a VALID (or VALIDATED) plan into a new output/run_YYYYMMDD_HHMMSS/ folder.
        ``require_visual=True`` accepts only plans that passed the optional visual guard."""
        from .. import __version__
        from ..export import export_plan
        return export_plan(plan, metadata, output_root or self.paths.output_dir,
                           buffer_m=self.config.aoi.buffer_m,
                           input_dir=self.photo_dir if self.config.photo_dir
                           else self.paths.input_dir,
                           tool_version=__version__, require_visual=require_visual)

def run(config: Optional[RunConfig] = None,
        progress: Optional[ProgressCallback] = None) -> SelectionResult:
    """Functional shortcut for ``UAVPhotoOptimizer(config).run()``."""
    return UAVPhotoOptimizer(config).run(progress)
