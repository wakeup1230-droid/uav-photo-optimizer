"""End-to-end UAVPhotoOptimizer.analyse_geometry() on a synthetic mission (ExifTool stubbed)."""

import pytest
from shapely.geometry import box

from conftest import make_project, write_shp
from synthetic_flight import X0, Y0, lawnmower
from uav_photo_optimizer import RunConfig, UAVPhotoOptimizer
from uav_photo_optimizer.core.config import FootprintProjectorKind
from uav_photo_optimizer.metadata.altitude import HeightStrategy
from uav_photo_optimizer.metadata.exiftool import MetadataReadResult
from uav_photo_optimizer.overlap.models import OverlapKind


def test_analyse_geometry_synthetic(tmp_path, monkeypatch):
    base = make_project(tmp_path)
    write_shp(base, geom=box(X0, Y0, X0 + 600, Y0 + 120))
    photos = lawnmower(strips=3, photo_spacing=30, strip_spacing=60)
    for p in photos:                                   # files only need to exist
        f = base / "input" / "photo" / p.photo_id
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_bytes(b"x")
    engine = UAVPhotoOptimizer(RunConfig.create(
        base_dir=base, footprint_projector=FootprintProjectorKind.PLANAR,
        height_strategy=HeightStrategy.TAKEOFF_RELATIVE, front_overlap=80, side_overlap=70))
    monkeypatch.setattr(engine.exiftool, "read",
                        lambda *a, **k: MetadataReadResult(photos=photos, errors=[]))

    g = engine.analyse_geometry()
    assert g.photos_scanned == len(photos) and len(g.footprints) == len(photos)
    assert len(g.flights.flights) == 1 and len(g.flights.strips) == 3
    front = g.overlap.results(OverlapKind.FRONT_OVERLAP)
    side = g.overlap.results(OverlapKind.SIDE_OVERLAP)
    assert all(e.value == pytest.approx(0.8, abs=1e-4) for e in front)
    assert all(e.value == pytest.approx(0.7, abs=1e-4) for e in side)
    assert g.statistics["FRONT_OVERLAP"]["pct"]["median"] == pytest.approx(80, abs=0.01)
    assert any("Evaluation only" in w for w in g.warnings)
    assert type(g).model_validate_json(g.model_dump_json()).photos_scanned == len(photos)
    # nothing in input changed, nothing written to output
    assert not any((base / "output").iterdir())
