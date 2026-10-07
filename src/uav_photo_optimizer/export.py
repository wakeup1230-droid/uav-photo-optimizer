"""
Consumer-side export (never part of the Core Engine). Nothing here deletes, moves, renames or
overwrites; input is never written.

* ``copy_selected``     — legacy GPS-selection copy (Phase 0/1)
* ``export_plan``       — COPY of a VALID (geometric) or VALIDATED SelectionPlan into a new
                          ``output/run_YYYYMMDD_HHMMSS/`` folder:
                              photos/                (flat, shutil.copy2)
                              selection_plan.json
                              selection_manifest.csv (audit trail)
                              run_summary.json       (= future REST API result model)
"""

from __future__ import annotations

import csv
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from .core.exceptions import ExportError
from .core.result import SelectionResult


@dataclass
class ExportResult:
    copied: list[Path] = field(default_factory=list)
    skipped: list[tuple[Path, str]] = field(default_factory=list)


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def new_run_dir(output_root: Path, now: datetime | None = None) -> Path:
    """Create and return a fresh ``output_root/run_YYYYMMDD_HHMMSS[_n]`` folder (never reused)."""
    stamp = (now or datetime.now()).strftime("run_%Y%m%d_%H%M%S")
    output_root.mkdir(parents=True, exist_ok=True)
    for n in range(1000):
        candidate = output_root / (stamp if n == 0 else f"{stamp}_{n}")
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise FileExistsError(f"cannot create a new run folder in {output_root}")


def copy_selected(result: SelectionResult, output_dir: Path,
                  input_dir: Path | None = None) -> ExportResult:
    """
    Copy ``result.selected_photos`` flat into ``output_dir`` (``shutil.copy2``).

    Duplicate filenames among the selection are all skipped; existing files in
    ``output_dir`` are never overwritten.
    """
    output_dir = Path(output_dir)
    if input_dir is not None and _is_within(output_dir, input_dir):
        raise ExportError(f"output 不可位於 input 內（input 為唯讀）：{output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    by_name: dict[str, list[Path]] = {}
    for rec in result.selected_photos:
        by_name.setdefault(rec.filename.lower(), []).append(Path(rec.path))

    out = ExportResult()
    for group in by_name.values():
        if len(group) > 1:
            out.skipped.extend((p, "檔名重複") for p in group)
            continue
        src = group[0]
        dst = output_dir / src.name
        if dst.exists():
            out.skipped.append((src, "output 已存在同名檔案"))
            continue
        try:
            shutil.copy2(src, dst)
            out.copied.append(dst)
        except OSError as exc:
            out.skipped.append((src, f"複製失敗：{exc}"))
    return out


# --- plan export (Phase 6B; visual optional since v1.0) ------------------------------------

MANIFEST_FIELDS = ["filename", "source_path", "decision", "reason", "flight_id", "strip_id",
                   "capture_type", "view_group", "front_or_along_metric",
                   "side_or_cross_metric", "visual_status", "visual_inliers",
                   "visual_inlier_ratio", "height_strategy", "height_confidence"]


class RunSummary(BaseModel):
    """Result summary of one validated export (future REST API result model)."""

    model_config = ConfigDict(extra="forbid")

    tool_version: str
    created_at: datetime
    run_dir: str
    buffer_m: float
    front_target: float
    side_target: float
    photos_scanned: int
    candidate_photos: int
    selected: int                 # KEEP + PROTECTED (= copied)
    removed: int
    protected: int
    geometric_restore_count: int
    visual_restore_count: int
    reduction_percent: float
    coverage_valid: bool
    geometry_valid: bool
    visual_valid: bool
    copied_files: int
    warnings: list[str] = Field(default_factory=list)


class ValidatedExportResult(BaseModel):
    model_config = ConfigDict(extra="forbid")

    run_dir: str
    photos_dir: str
    copied: int
    collisions: list[str] = Field(default_factory=list)
    errors: list[str] = Field(default_factory=list)
    plan_path: str
    manifest_path: str
    summary_path: str
    summary: RunSummary


def export_validated(plan, metadata: dict, output_root: Path, *, buffer_m: float,
                     input_dir: Optional[Path] = None, tool_version: str = "",
                     now: Optional[datetime] = None) -> ValidatedExportResult:
    """COPY the selected photos of a VALIDATED plan (visual guard required)."""
    return export_plan(plan, metadata, output_root, buffer_m=buffer_m, input_dir=input_dir,
                       tool_version=tool_version, now=now, require_visual=True)


def export_plan(plan, metadata: dict, output_root: Path, *, buffer_m: float,
                input_dir: Optional[Path] = None, tool_version: str = "",
                now: Optional[datetime] = None,
                require_visual: bool = False) -> ValidatedExportResult:
    """
    COPY the selected (KEEP + PROTECTED) photos of a plan into a new run folder.

    The official pipeline exports a geometrically VALID plan (the visual guard is optional,
    default OFF). With ``require_visual`` only VALIDATED plans are accepted. An INVALID plan is
    never exported. Raises ExportError before copying anything when filenames collide, a
    source is missing or the target is unsafe.
    """
    from .optimizer.models import DecisionStatus, PlanStatus

    allowed = ({PlanStatus.VALIDATED} if require_visual
               else {PlanStatus.VALID, PlanStatus.VALIDATED})
    if plan.status not in allowed or plan.safety.total != 0:
        need = "VALIDATED" if require_visual else "VALID or VALIDATED"
        raise ExportError(f"plan status {plan.status.value}: only {need} plans may be exported")
    output_root = Path(output_root)
    if input_dir is not None and _is_within(output_root, input_dir):
        raise ExportError(f"output 不可位於 input 內（input 為唯讀）：{output_root}")
    selected = [d for d in plan.decisions if d.status is not DecisionStatus.REMOVE]
    names: dict[str, list[str]] = {}
    for d in selected:
        names.setdefault(Path(metadata[d.photo_id].path).name.lower(), []).append(d.photo_id)
    collisions = sorted(p for ids in names.values() if len(ids) > 1 for p in ids)
    if collisions:
        raise ExportError(f"{len(collisions)} selected photos share a filename; nothing copied: "
                          + ", ".join(collisions[:5]))
    missing = [d.photo_id for d in selected if not Path(metadata[d.photo_id].path).is_file()]
    if missing:
        raise ExportError(f"{len(missing)} source photos not found; nothing copied")

    run_dir = new_run_dir(output_root, now)
    photos_dir = run_dir / "photos"
    photos_dir.mkdir()
    copied = 0
    for d in selected:
        src = Path(metadata[d.photo_id].path)
        dst = photos_dir / src.name
        if dst.exists():                                   # never overwrite
            raise ExportError(f"destination exists, export stopped: {dst}")
        shutil.copy2(src, dst)
        copied += 1

    plan_path = run_dir / "selection_plan.json"
    plan_path.write_text(plan.model_dump_json(indent=1), encoding="utf-8")
    manifest_path = run_dir / "selection_manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=MANIFEST_FIELDS)
        w.writeheader()
        for d in plan.decisions:
            det = d.detail
            w.writerow({"filename": Path(metadata[d.photo_id].path).name,
                        "source_path": str(metadata[d.photo_id].path),
                        "decision": d.status.value, "reason": d.reason.value,
                        "flight_id": d.flight_id, "strip_id": d.strip_id,
                        "capture_type": d.capture_type, "view_group": d.view_group,
                        "front_or_along_metric": det.get("along_metric_pct"),
                        "side_or_cross_metric": det.get("cross_metric_pct"),
                        "visual_status": det.get("visual_status"),
                        "visual_inliers": det.get("visual_inliers"),
                        "visual_inlier_ratio": det.get("visual_inlier_ratio"),
                        "height_strategy": det.get("height_strategy"),
                        "height_confidence": det.get("height_confidence")})
    summary = RunSummary(
        tool_version=tool_version, created_at=datetime.now(), run_dir=str(run_dir),
        buffer_m=buffer_m, front_target=plan.config.front_target_pct,
        side_target=plan.config.side_target_pct, photos_scanned=plan.input_photo_count,
        candidate_photos=plan.candidate_photo_count, selected=len(selected),
        removed=plan.remove_count, protected=plan.protected_count,
        geometric_restore_count=plan.restored_count,
        visual_restore_count=plan.visual_restore_count,
        reduction_percent=plan.reduction_percent,
        coverage_valid=plan.safety.new_coverage_holes == 0,
        geometry_valid=bool(plan.geometry_valid), visual_valid=bool(plan.visual_valid),
        copied_files=copied, warnings=plan.warnings)
    summary_path = run_dir / "run_summary.json"
    summary_path.write_text(summary.model_dump_json(indent=1), encoding="utf-8")
    return ValidatedExportResult(run_dir=str(run_dir), photos_dir=str(photos_dir), copied=copied,
                                 plan_path=str(plan_path), manifest_path=str(manifest_path),
                                 summary_path=str(summary_path), summary=summary)
