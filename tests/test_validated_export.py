"""Phase 8A visual guard (fake validator) + Phase 6B validated export."""

import csv
import hashlib
import json
from pathlib import Path

import pytest
from shapely.geometry import box

from conftest import make_project, write_shp
from synthetic_flight import X0, Y0, lawnmower
from uav_photo_optimizer import RunConfig, UAVPhotoOptimizer
from uav_photo_optimizer.core.config import FootprintProjectorKind
from uav_photo_optimizer.core.exceptions import ExportError
from uav_photo_optimizer.optimizer.models import (DecisionStatus, PlanStatus,
                                                  SelectionOptimizerConfig, SelectionReason)
from uav_photo_optimizer.visual.base import VisualValidator
from uav_photo_optimizer.visual.models import VisualMatchResult, VisualStatus


class SeqValidator(VisualValidator):
    """PASS when the two photos are at most ``k`` capture positions apart."""

    name = "SeqValidator"

    def __init__(self, k=None, status_far=VisualStatus.FAIL):
        self.k, self.status_far, self.calls = k, status_far, []

    def validate_pair(self, a, b, context):
        self.calls.append((a.photo_id, b.photo_id))
        sa, sb = (int(Path(p.photo_id).stem.split("_")[2]) for p in (a, b))
        ok = self.k is None or abs(sa - sb) <= self.k
        return VisualMatchResult(photo_a=a.photo_id, photo_b=b.photo_id, detector="fake",
                                 matcher="fake", essential_inliers=500 if ok else 3,
                                 status=VisualStatus.PASS if ok else self.status_far)


@pytest.fixture
def mission(tmp_path):
    base = make_project(tmp_path)
    write_shp(base, geom=box(X0 + 100, Y0 - 50, X0 + 500, Y0 + 170))
    photos = []
    for m in lawnmower(strips=3, photo_spacing=10):
        f = base / "input" / "photo" / m.photo_id
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_bytes(m.photo_id.encode() * 50)               # small stand-in file
        photos.append(m.model_copy(update={"path": f}))
    eng = UAVPhotoOptimizer(RunConfig.create(base_dir=base, buffer_m=0, front_overlap=65,
                                             side_overlap=65,
                                             footprint_projector=FootprintProjectorKind.PLANAR))
    g = eng.geometry_from_metadata(photos, len(photos))
    plan = eng.plan_selection(SelectionOptimizerConfig(front_target_pct=65, side_target_pct=65),
                              geometry=g)
    return base, eng, g, plan, {m.photo_id: m for m in photos}


def test_geometric_plan_exportable_visual_optional(mission):
    """v1.0: the visual guard is optional (default OFF) — a VALID plan may be copied, but not
    when visual validation is explicitly required."""
    base, eng, g, plan, meta = mission
    assert plan.status is PlanStatus.VALID and plan.exportable and plan.remove_count > 0
    with pytest.raises(ExportError, match="VALIDATED"):
        eng.export_selection(plan, meta, require_visual=True)
    assert not any((base / "output").iterdir())
    res = eng.export_selection(plan, meta)
    assert res.copied == plan.keep_count + plan.protected_count
    assert not res.summary.visual_valid


def test_invalid_plan_never_exported(mission):
    base, eng, g, plan, meta = mission
    bad = plan.model_copy(update={"status": PlanStatus.INVALID, "exportable": False})
    with pytest.raises(ExportError):
        eng.export_selection(bad, meta)


def test_all_new_edges_validated_and_pass(mission):
    _, eng, g, plan, meta = mission
    v = SeqValidator(k=None)
    vp = eng.validate_selection(plan, g, meta, validator=v)
    rep = vp.visual
    assert vp.status is PlanStatus.VALIDATED and vp.exportable
    assert vp.remove_count == plan.remove_count and vp.visual_restore_count == 0
    assert rep["new_edges"] > 0 and rep["new_edges_pass"] == rep["new_edges"]
    validated = {tuple(c) for c in v.calls}
    new = {(e["photo_a"], e["photo_b"]) for e in rep["edges"] if e["kind"] == "NEW_EDGE"}
    assert new <= validated


def test_visual_fail_restores_middle_photos(mission):
    _, eng, g, plan, meta = mission
    vp = eng.validate_selection(plan, g, meta, validator=SeqValidator(k=2))
    assert vp.status is PlanStatus.VALIDATED
    assert vp.visual_restore_count > 0
    assert vp.remove_count < plan.remove_count
    assert vp.visual["remaining_new_failed_edges"] == 0
    assert any(d.reason is SelectionReason.RESTORED_BY_VISUAL_VALIDATION for d in vp.decisions)
    assert vp.safety.total == 0 and vp.geometry_valid and vp.visual_valid
    # every remaining new edge spans at most 2 capture positions
    for e in vp.visual["edges"]:
        if e["kind"] == "NEW_EDGE":
            assert e["result"]["status"] == "PASS"


def test_visual_unresolved_restores_conservatively(mission):
    _, eng, g, plan, meta = mission
    vp = eng.validate_selection(plan, g, meta,
                                validator=SeqValidator(k=0, status_far=VisualStatus.UNRESOLVED))
    assert vp.status is PlanStatus.VALIDATED
    assert vp.remove_count == 0                       # every removal undone
    assert vp.visual["new_edges_unresolved"] > 0


def test_unresolved_height_stays_protected(tmp_path):
    base = make_project(tmp_path)
    write_shp(base, geom=box(X0 + 100, Y0 - 50, X0 + 500, Y0 + 170))
    photos = lawnmower(strips=1, photo_spacing=10)
    for i in range(16, 27):
        photos[i] = photos[i].model_copy(update={"lrf_status": "Error"})
    eng = UAVPhotoOptimizer(RunConfig.create(base_dir=base, buffer_m=0,
                                             footprint_projector=FootprintProjectorKind.PLANAR))
    g = eng.geometry_from_metadata(photos, len(photos))
    plan = eng.plan_selection(geometry=g)
    vp = eng.validate_selection(plan, g, {m.photo_id: m for m in photos},
                                validator=SeqValidator(k=None))
    by = {d.photo_id: d for d in vp.decisions}
    assert by[photos[21].photo_id].reason is SelectionReason.PROTECTED_HEIGHT_UNRESOLVED
    assert by[photos[21].photo_id].detail["height_strategy"] == "HEIGHT_UNRESOLVED"


def sha(p):
    return hashlib.sha1(Path(p).read_bytes()).hexdigest()


def test_validated_export(mission):
    base, eng, g, plan, meta = mission
    hashes = {pid: sha(m.path) for pid, m in meta.items()}
    vp = eng.validate_selection(plan, g, meta, validator=SeqValidator(k=2))
    res = eng.export_selection(vp, meta)
    run = Path(res.run_dir)
    assert run.parent == base / "output" and run.name.startswith("run_")
    copied = sorted(p.name for p in (run / "photos").iterdir())
    selected = sorted(Path(meta[d.photo_id].path).name for d in vp.decisions
                      if d.status is not DecisionStatus.REMOVE)
    assert copied == selected and res.copied == len(selected)
    rows = list(csv.DictReader((run / "selection_manifest.csv").open(encoding="utf-8-sig")))
    assert len(rows) == vp.candidate_photo_count
    assert {"filename", "source_path", "decision", "reason", "visual_status",
            "front_or_along_metric", "side_or_cross_metric", "height_strategy",
            "height_confidence"} <= set(rows[0])
    summary = json.loads((run / "run_summary.json").read_text(encoding="utf-8"))
    assert summary["visual_valid"] and summary["geometry_valid"] and summary["coverage_valid"]
    assert summary["copied_files"] == len(selected) and summary["visual_restore_count"] > 0
    assert (run / "selection_plan.json").is_file()
    # second export → a new run folder, nothing overwritten
    res2 = eng.export_selection(vp, meta)
    assert res2.run_dir != res.run_dir
    # input untouched
    assert {pid: sha(m.path) for pid, m in meta.items()} == hashes


def test_export_collision_stops_before_copy(mission):
    base, eng, g, plan, meta = mission
    vp = eng.validate_selection(plan, g, meta, validator=SeqValidator(k=None))
    sel = [d.photo_id for d in vp.decisions if d.status is not DecisionStatus.REMOVE]
    clash = dict(meta)
    clash[sel[1]] = meta[sel[1]].model_copy(update={"path": meta[sel[0]].path.with_name(
        Path(meta[sel[0]].path).name)})
    with pytest.raises(ExportError, match="share a filename"):
        eng.export_selection(vp, clash)
    assert not any((base / "output").iterdir())
