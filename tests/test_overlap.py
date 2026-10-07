"""
Overlap engine on synthetic missions with analytic answers, plus property tests (C12).

Footprint 200 m (cross) × 150 m (along):  front = 1 − d / 150,  side = 1 − s / 200.
"""

import pytest
from shapely.geometry import box

from synthetic_flight import lawnmower
from uav_photo_optimizer.camera.intrinsics import resolve_intrinsics
from uav_photo_optimizer.flight.strip import analyse_flights
from uav_photo_optimizer.footprint.base import ProjectionContext
from uav_photo_optimizer.footprint.planar import PlanarProjector
from uav_photo_optimizer.metadata.altitude import HeightStrategy
from uav_photo_optimizer.overlap.engine import OverlapConfig, OverlapEngine, overlap_statistics
from uav_photo_optimizer.overlap.front import along_track_overlap
from uav_photo_optimizer.overlap.models import OverlapKind, OverlapMethod, TerrainMode
from uav_photo_optimizer.overlap.side import cross_track_overlap

CRS = "EPSG:3826"
CTX = ProjectionContext(target_crs=CRS, height_strategy=HeightStrategy.TAKEOFF_RELATIVE)


def run(photos, config=OverlapConfig()):
    a = analyse_flights(photos, CRS)
    proj = PlanarProjector()
    fps = {m.photo_id: proj.project(m, resolve_intrinsics(m), CTX) for m in photos}
    rep = OverlapEngine(config).compute(fps, a, {m.photo_id: m.capture_time_utc for m in photos})
    return a, fps, rep


def values(rep, kind):
    return [e.value for e in rep.results(kind)]


def test_exact_front_and_side():
    a, fps, rep = run(lawnmower(strips=3, photo_spacing=30, strip_spacing=60))
    front, side = values(rep, OverlapKind.FRONT_OVERLAP), values(rep, OverlapKind.SIDE_OVERLAP)
    assert front and all(v == pytest.approx(1 - 30 / 150, abs=1e-4) for v in front)   # 80 %
    assert side and all(v == pytest.approx(1 - 60 / 200, abs=1e-4) for v in side)     # 70 %
    e = rep.results(OverlapKind.FRONT_OVERLAP)[0]
    assert e.axis.length_a == pytest.approx(150, abs=1e-3)
    assert e.axis.overlap_length == pytest.approx(120, abs=1e-3)
    assert e.geometry_method is OverlapMethod.NADIR_GEOMETRIC_PLANAR
    assert e.terrain_mode is TerrainMode.LOCAL_HORIZONTAL_PLANE
    assert "TERRAIN_NOT_ACCOUNTED_FOR" in e.warnings
    assert e.height_strategy == "TAKEOFF_RELATIVE" and e.same_strip
    # shared coverage is reported but differs from the directional metric
    assert e.shared.iou == pytest.approx(120 / 180, abs=1e-6)
    assert e.shared.over_min_area == pytest.approx(0.8, abs=1e-6)


def test_target_evaluation_only():
    photos = lawnmower(strips=2, photo_spacing=30, strip_spacing=60)
    _, _, rep = run(photos, OverlapConfig(front_target_pct=85, side_target_pct=65))
    assert all(e.target_pass is False for e in rep.results(OverlapKind.FRONT_OVERLAP))  # 80<85
    assert all(e.target_pass is True for e in rep.results(OverlapKind.SIDE_OVERLAP))    # 70≥65
    assert len(rep.graph.nodes) == len(photos)          # nothing removed


@pytest.mark.parametrize("d1, d2", [(15, 30), (30, 45), (45, 75)])
def test_property_front_decreases_with_spacing(d1, d2):
    f1 = values(run(lawnmower(strips=2, photo_spacing=d1))[2], OverlapKind.FRONT_OVERLAP)
    f2 = values(run(lawnmower(strips=2, photo_spacing=d2))[2], OverlapKind.FRONT_OVERLAP)
    assert max(f2) < min(f1)


@pytest.mark.parametrize("s1, s2", [(40, 60), (60, 100), (100, 150)])
def test_property_side_decreases_with_strip_spacing(s1, s2):
    v1 = values(run(lawnmower(strips=3, strip_spacing=s1))[2], OverlapKind.SIDE_OVERLAP)
    v2 = values(run(lawnmower(strips=3, strip_spacing=s2))[2], OverlapKind.SIDE_OVERLAP)
    assert max(v2) < min(v1)


def test_property_higher_altitude_more_overlap():
    lo = values(run(lawnmower(strips=2, height=100))[2], OverlapKind.FRONT_OVERLAP)
    hi = values(run(lawnmower(strips=2, height=150))[2], OverlapKind.FRONT_OVERLAP)
    assert min(hi) > max(lo)
    assert hi[0] == pytest.approx(1 - 30 / 225, abs=1e-4)


def test_rank2_pairs():
    _, _, rep = run(lawnmower(strips=1, photo_spacing=30))
    r2 = rep.results(OverlapKind.FRONT_OVERLAP, neighbour_rank=2)
    assert r2 and all(e.value == pytest.approx(1 - 60 / 150, abs=1e-4) for e in r2)


def test_strip_pairing_by_adjacency_not_id():
    photos = lawnmower(strips=3, strip_spacing=60)
    a, fps, rep = run(photos)
    pairs = {(p.strip_a, p.strip_b) for p in rep.strip_pairs}
    s1, s2, s3 = (s.strip_id for s in a.strips)
    assert (s1, s2) in pairs and (s2, s3) in pairs and (s1, s3) not in pairs
    # a second mission flown later between the lines pairs by geometry, across flights
    from datetime import timedelta
    extra = lawnmower(strips=1, folder="M2", y0=photos[0].latitude and 2655030.0,
                      start=photos[-1].capture_time + timedelta(seconds=600))
    a2, _, rep2 = run(photos + extra)
    pairs2 = {(p.strip_a, p.strip_b) for p in rep2.strip_pairs}
    new = a2.strips[-1].strip_id
    assert (s1, new) in pairs2 or (new, s1) in pairs2
    assert any(not p.same_flight for p in rep2.strip_pairs)


def test_oblique_kinds_and_method():
    _, _, rep = run(lawnmower(strips=2, oblique_every=1))
    along = rep.results(OverlapKind.ALONG_TRACK_GEOMETRIC_OVERLAP)
    cross = rep.results(OverlapKind.CROSS_TRACK_GEOMETRIC_OVERLAP)
    assert along and all(e.geometry_method is OverlapMethod.OBLIQUE_GEOMETRIC_PLANAR
                         for e in along)
    # forward-left in a 90° strip looks to 45°, in the 270° strip to 225°: different look
    # sectors → no cross-track oblique pairs between them
    assert not cross


def test_single_pair_functions():
    a, b = box(0, 0, 200, 150), box(0, 30, 200, 180)        # heading 0 → along = y
    f = along_track_overlap(a, b, 0.0)
    assert f.over_max == pytest.approx(0.8) and f.over_min == pytest.approx(0.8)
    s = cross_track_overlap(a, box(60, 0, 260, 150), 0.0)
    assert s.over_max == pytest.approx(0.7)
    small = along_track_overlap(box(0, 0, 200, 150), box(0, 30, 200, 130), 0.0)
    assert small.over_min == pytest.approx(1.0) and small.over_max == pytest.approx(100 / 150)


def test_statistics_shape():
    a, _, rep = run(lawnmower(strips=3))
    st = overlap_statistics(rep, a)
    fr = st["FRONT_OVERLAP"]
    assert fr["pct"]["median"] == pytest.approx(80, abs=1e-2)
    assert set(fr["pct"]) >= {"min", "p05", "p25", "median", "p75", "p95", "max"}
    assert fr["pass_rate"] == 1.0 and fr["by_flight"] and fr["by_strip"]


def test_graph_data_only():
    _, _, rep = run(lawnmower(strips=2))
    deg = rep.graph.degree()
    pid = rep.graph.nodes[3]
    assert deg[pid] == len(rep.graph.neighbours(pid)) > 0
