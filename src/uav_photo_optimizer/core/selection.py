"""
Official photo selection pipeline (v1.0 core scope).

    AOI → Buffer → Photo Metadata → Planar / LRF Estimated Footprint → Flight Strip
        → Overlap Analysis → Optimizer → COPY

This is the one entry point used by the CLI (``select``), the REST API and the GUI. The
tool gives a *geometric photo selection recommendation*; it does not guarantee that the
photogrammetric reconstruction succeeds.

Six user parameters only: AOI shapefile, photo folder, buffer, front overlap, side overlap,
output folder. Everything else (CRS, ExifTool, LRF / height strategy, lens model, flight-strip
parameters) is resolved internally. Visual Guard and Terrain are optional research features
(``AdvancedOptions``), default OFF, and never required.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from pydantic import BaseModel, ConfigDict, Field

from .config import (DEFAULT_BUFFER_M, DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP,
                     MAX_OVERLAP, MIN_BUFFER_M, MIN_OVERLAP, RunConfig)
from .exceptions import ConfigError, ExportError

ORIGINAL_DATA_NOTE = "部分原始照片重疊率低於設定值。"
ORIGINAL_DATA_DETAIL = "篩選結果已保留可用照片，但無法改善原始拍攝資料。"

Progress = Callable[[str, int, int], None]


class SelectionRequest(BaseModel):
    """The six user parameters (+ whether to copy)."""

    model_config = ConfigDict(extra="forbid")

    aoi_shapefile: Path
    photo_dir: Path
    output_dir: Optional[Path] = None        # required when copy=True
    buffer_m: float = Field(DEFAULT_BUFFER_M, ge=MIN_BUFFER_M)
    front_overlap: float = Field(DEFAULT_FRONT_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    side_overlap: float = Field(DEFAULT_SIDE_OVERLAP, ge=MIN_OVERLAP, le=MAX_OVERLAP)
    copy_photos: bool = True


class AdvancedOptions(BaseModel):
    """Experimental research features. Default OFF; not part of the public API v1."""

    model_config = ConfigDict(extra="forbid")

    visual_guard: bool = False               # needs the optional extra `visual` (OpenCV)


class PhotoItem(BaseModel):
    model_config = ConfigDict(extra="forbid")

    filename: str
    path: str


class SelectionSummary(BaseModel):
    """Result of one selection (= REST API v1 result)."""

    model_config = ConfigDict(extra="forbid")

    candidate_photos: int
    selected_photos: int
    removed_photos: int
    reduction_percent: float
    selected: list[PhotoItem] = Field(default_factory=list)
    removed: list[PhotoItem] = Field(default_factory=list)
    original_overlap_below_target: bool = False
    notes: list[str] = Field(default_factory=list)
    output_dir: Optional[str] = None         # run folder that received the copies
    copied_photos: int = 0
    photos_scanned: int = 0
    buffer_m: float
    front_overlap: float
    side_overlap: float
    tool_version: str = ""
    started_at: Optional[datetime] = None
    finished_at: Optional[datetime] = None


def _within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def validate_request(req: SelectionRequest) -> None:
    if not req.aoi_shapefile.is_file():
        raise ConfigError(f"找不到建模範圍 Shapefile：{req.aoi_shapefile}")
    if req.aoi_shapefile.suffix.lower() != ".shp":
        raise ConfigError(f"建模範圍必須是 .shp 檔：{req.aoi_shapefile}")
    if not req.photo_dir.is_dir():
        raise ConfigError(f"找不到照片資料夾：{req.photo_dir}")
    if req.copy_photos:
        if req.output_dir is None:
            raise ConfigError("請指定輸出資料夾")
        if _within(req.output_dir, req.photo_dir):
            raise ExportError("輸出資料夾不可位於照片資料夾內（照片資料夾為唯讀）")


def select_photos(req: SelectionRequest, progress: Optional[Progress] = None,
                  advanced: Optional[AdvancedOptions] = None,
                  base_dir: Optional[Path] = None) -> SelectionSummary:
    """Run the official pipeline. Photos are only COPIED (never moved / deleted / overwritten)
    into a new ``<output_dir>/run_YYYYMMDD_HHMMSS/`` folder."""
    from .. import __version__
    from ..export import export_plan
    from ..optimizer.models import DecisionStatus, DefectType, SelectionOptimizerConfig
    from .engine import UAVPhotoOptimizer

    advanced = advanced or AdvancedOptions()
    validate_request(req)
    started = datetime.now()

    def step(stage: str, done: int = 0, total: int = 0) -> None:
        if progress:
            progress(stage, done, total)

    cfg = RunConfig.create(buffer_m=req.buffer_m, front_overlap=req.front_overlap,
                           side_overlap=req.side_overlap, shapefile=req.aoi_shapefile,
                           photo_dir=req.photo_dir, base_dir=base_dir)
    eng = UAVPhotoOptimizer(cfg)
    from ..metadata.scanner import find_photos
    step("scan")
    photos = find_photos(eng.photo_dir)
    read = eng.exiftool.read(photos, photo_root=eng.photo_dir,
                             progress=lambda d, t: step("metadata", d, t))
    meta = {m.photo_id: m for m in read.photos}
    step("geometry")
    geometry = eng.geometry_from_metadata(read.photos, len(photos))
    step("optimize")
    plan = eng.plan_selection(SelectionOptimizerConfig(front_target_pct=req.front_overlap,
                                                       side_target_pct=req.side_overlap),
                              geometry=geometry)
    if advanced.visual_guard:
        step("visual")
        plan = eng.validate_selection(plan, geometry, meta)

    gaps = {DefectType.FRONT_GAP, DefectType.SIDE_GAP, DefectType.ALONG_TRACK_GAP,
            DefectType.CROSS_TRACK_GAP, DefectType.AOI_COVERAGE_GAP}
    below = any(d.type in gaps for d in plan.baseline_defects)
    notes = [ORIGINAL_DATA_NOTE, ORIGINAL_DATA_DETAIL] if below else []

    def item(pid: str) -> PhotoItem:
        p = Path(meta[pid].path)
        return PhotoItem(filename=p.name, path=str(p))

    selected = [item(d.photo_id) for d in plan.decisions if d.status is not DecisionStatus.REMOVE]
    removed = [item(d.photo_id) for d in plan.decisions if d.status is DecisionStatus.REMOVE]
    out_dir, copied = None, 0
    if req.copy_photos:
        step("copy", 0, len(selected))
        res = export_plan(plan, meta, req.output_dir, buffer_m=req.buffer_m,
                          input_dir=req.photo_dir, tool_version=__version__,
                          require_visual=advanced.visual_guard)
        out_dir, copied = res.run_dir, res.copied
        step("copy", copied, len(selected))
    step("done")
    return SelectionSummary(
        candidate_photos=plan.candidate_photo_count, selected_photos=len(selected),
        removed_photos=len(removed), reduction_percent=round(plan.reduction_percent, 1),
        selected=selected, removed=removed, original_overlap_below_target=below, notes=notes,
        output_dir=out_dir, copied_photos=copied, photos_scanned=len(photos),
        buffer_m=req.buffer_m, front_overlap=req.front_overlap, side_overlap=req.side_overlap,
        tool_version=__version__, started_at=started, finished_at=datetime.now())
