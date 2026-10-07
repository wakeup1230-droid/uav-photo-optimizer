"""Phase 6A optimizer (dry run) on synthetic missions."""

import pytest
from shapely.geometry import box

from conftest import make_project, write_shp
from synthetic_flight import X0, Y0, lawnmower
from uav_photo_optimizer import RunConfig, UAVPhotoOptimizer
from uav_photo_optimizer.core.config import FootprintProjectorKind
from uav_photo_optimizer.optimizer.models import (DecisionStatus, DefectType, PlanStatus,
                                                  SelectionOptimizerConfig, SelectionPlan,
                                                  SelectionReason)
from uav_photo_optimizer.overlap.front import along_track_overlap


def setup(tmp_path, photos, aoi=None, buffer_m=0):
    base = make_project(tmp_path)
    write_shp(base, geom=aoi or box(X0 + 100, Y0 - 50, X0 + 500, Y0 + 170))
    eng = UAVPhotoOptimizer(RunConfig.create(base_dir=base, buffer_m=buffer_m,
                                             footprint_projector=FootprintProjectorKind.PLANAR))
    g = eng.geometry_from_metadata(photos, len(photos))
    return base, eng, g


def plan(eng, g, front=80, side=70, **kw):
    return eng.plan_selection(SelectionOptimizerConfig(front_target_pct=front,
                                                       side_target_pct=side, **kw), geometry=g)


def kept_ids(p):
    return {d.photo_id for d in p.decisions if d.status is not DecisionStatus.REMOVE}


def test_dry_run_valid_and_reduces(tmp_path):
    photos = lawnmower(strips=3, photo_spacing=10)            # front 93 %
    base, eng, g = setup(tmp_path, photos)
    p = plan(eng, g)
    assert p.status is PlanStatus.VALID and p.constraints_passed and p.dry_run
    assert p.safety.total == 0 and p.remove_count > 0
    assert p.keep_count + p.remove_count + p.protected_count == p.candidate_photo_count
    assert p.coverage_after.by_group_m2 == p.coverage_before.by_group_m2
    # every remaining consecutive nadir pair still meets the target
    kept = kept_ids(p)
    for s in g.flights.strips:
        ks = [pid for pid in s.photo_ids if pid in kept and pid in g.footprints]
        for a, b in zip(ks, ks[1:]):
            v = along_track_overlap(g.footprints[a].geometry, g.footprints[b].geometry,
                                    s.travel_heading).over_max * 100
            assert v >= 80 - 1e-3
    assert not any((base / "output").iterdir())                # nothing copied


def test_strip_endpoints_kept_and_reasons(tmp_path):
    _, eng, g = setup(tmp_path, lawnmower(strips=1, photo_spacing=10))
    p = plan(eng, g)
    by = {d.photo_id: d for d in p.decisions}
    strip = g.flights.strips[0]
    cand = [pid for pid in strip.photo_ids if pid in by]
    assert by[cand[0]].status is not DecisionStatus.REMOVE
    assert by[cand[-1]].status is not DecisionStatus.REMOVE
    assert {d.reason for d in p.decisions if d.status is DecisionStatus.REMOVE} == \
        {SelectionReason.REMOVE_REDUNDANT_ALONG_TRACK}


def test_baseline_gap_protected_not_bridged(tmp_path):
    photos = lawnmower(strips=1, photo_spacing=10)
    gap = [p for i, p in enumerate(photos) if not 25 <= i <= 37]   # 140 m hole mid-strip
    _, eng, g = setup(tmp_path, gap)
    p = plan(eng, g)
    fronts = [d for d in p.baseline_defects if d.type is DefectType.FRONT_GAP]
    assert len(fronts) == 1
    by = {d.photo_id: d for d in p.decisions}
    for pid in fronts[0].affected_photos:
        assert by[pid].status is DecisionStatus.PROTECTED
        assert by[pid].reason is SelectionReason.PROTECTED_BASELINE_DEFECT
    assert p.status is PlanStatus.VALID and p.safety.baseline_defects_worsened == 0


def test_monotonic_lower_target_more_reduction(tmp_path):
    _, eng, g = setup(tmp_path, lawnmower(strips=3, photo_spacing=10, oblique_every=1))
    reds = [plan(eng, g, f, s).remove_count for f, s in ((80, 70), (75, 70), (70, 65), (65, 65))]
    assert reds == sorted(reds)
    assert all(plan(eng, g, f, s).status is PlanStatus.VALID
               for f, s in ((80, 70), (65, 65)))


def test_whole_strip_removal_when_neighbours_overlap(tmp_path):
    # strips 20 m apart, footprint width 200 m: strips 1 & 3 (40 m apart) overlap 80 %
    photos = lawnmower(strips=3, photo_spacing=30, strip_spacing=20)
    _, eng, g = setup(tmp_path, photos, aoi=box(X0 + 100, Y0 - 50, X0 + 500, Y0 + 90))
    p = plan(eng, g, 80, 70)
    middle = g.flights.strips[1].strip_id
    ev = {e.strip_id: e for e in p.strip_evaluations}[middle]
    assert ev.neighbours_side_overlap_min_pct == pytest.approx(80.0, abs=0.01)
    assert ev.removable and ev.removed
    assert any(d.reason is SelectionReason.REMOVE_REDUNDANT_CROSS_TRACK for d in p.decisions)
    assert p.status is PlanStatus.VALID


def test_unresolved_height_protected(tmp_path):
    photos = lawnmower(strips=1, photo_spacing=10)
    for i in range(16, 27):        # 11 invalid in a row → photo 21 is ≥ 60 m from valid LRF
        photos[i] = photos[i].model_copy(update={"lrf_status": "Error"})
    _, eng, g = setup(tmp_path, photos)
    p = plan(eng, g)
    by = {d.photo_id: d for d in p.decisions}
    assert by[photos[21].photo_id].reason is SelectionReason.PROTECTED_HEIGHT_UNRESOLVED
    assert by[photos[21].photo_id].status is DecisionStatus.PROTECTED
    assert p.status is PlanStatus.VALID


def test_oblique_groups_not_mixed(tmp_path):
    _, eng, g = setup(tmp_path, lawnmower(strips=2, photo_spacing=10, oblique_every=1))
    p = plan(eng, g, 65, 65)
    removed_obl = [d for d in p.decisions if d.status is DecisionStatus.REMOVE
                   and d.capture_type == "OBLIQUE"]
    assert removed_obl
    assert all(d.reason is SelectionReason.REMOVE_REDUNDANT_OBLIQUE_ALONG_TRACK
               for d in removed_obl)
    groups = {d.view_group for d in p.decisions if d.capture_type == "OBLIQUE"}
    assert len(groups) == 2                                # one look sector per strip direction


def test_plan_json_roundtrip(tmp_path):
    _, eng, g = setup(tmp_path, lawnmower(strips=2, photo_spacing=10))
    p = plan(eng, g)
    q = SelectionPlan.model_validate_json(p.model_dump_json())
    assert q.remove_count == p.remove_count and q.decisions == p.decisions


def test_config_validation_shared():
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        SelectionOptimizerConfig(front_target_pct=60)
    with pytest.raises(ValidationError):
        SelectionOptimizerConfig(side_target_pct=96)
