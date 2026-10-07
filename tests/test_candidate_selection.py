import math
from pathlib import Path

import geopandas as gpd
import pytest

from conftest import SQUARE, X0, X1, Y0, Y1, east_of_square, sha1, to_lonlat, write_jpg
from uav_photo_optimizer import (CandidateSelectionMode, PhotoReason, RunConfig,
                                 SelectionResult, UAVPhotoOptimizer)
from uav_photo_optimizer.aoi.buffer import build_buffer
from uav_photo_optimizer.aoi.selection import select_by_gps_point
from uav_photo_optimizer.core.exceptions import ExportError
from uav_photo_optimizer.export import copy_selected
from uav_photo_optimizer.metadata.models import PhotoMetadata


def _meta(name, x, y):
    lon, lat = to_lonlat(x, y)
    return PhotoMetadata(photo_id=name, filename=name, path=Path(name),
                         latitude=lat, longitude=lon)


def test_select_by_gps_point_pure():
    aoi = build_buffer(gpd.GeoDataFrame(geometry=[SQUARE], crs="EPSG:3826"), 100)
    photos = [
        _meta("inside", (X0 + X1) / 2, (Y0 + Y1) / 2),
        _meta("50m", *east_of_square(50)),
        _meta("99.9m", *east_of_square(99.9)),
        _meta("100.5m", *east_of_square(100.5)),
        _meta("150m", *east_of_square(150)),
    ]
    inside, outside = select_by_gps_point(photos, aoi)
    assert [p.photo_id for p in inside] == ["inside", "50m", "99.9m"]
    assert [p.photo_id for p in outside] == ["100.5m", "150m"]


def test_select_empty():
    aoi = build_buffer(gpd.GeoDataFrame(geometry=[SQUARE], crs="EPSG:3826"), 100)
    assert select_by_gps_point([], aoi) == ([], [])


@pytest.fixture
def standard_project(project):
    photo = project / "input" / "photo"
    files = {
        "inside": write_jpg(photo / "A" / "T01_INSIDE.JPG", (X0 + X1) / 2, (Y0 + Y1) / 2),
        "50m":    write_jpg(photo / "A" / "T02_50M.jpg", *east_of_square(50)),
        "99.9m":  write_jpg(photo / "B" / "T03_99_9M.jpeg", *east_of_square(99.9)),
        "150m":   write_jpg(photo / "B" / "T04_150M.JPEG", *east_of_square(150)),
        "no_gps": write_jpg(photo / "B" / "T05_NOGPS.JPG"),
        "level2": write_jpg(photo / "A" / "A1" / "T06_L2.JPG", X0 + 10, Y0 + 10),
        "deep":   write_jpg(photo / "C" / "C1" / "C2" / "子資料夾 中文" / "T07_DEEP.jpg",
                            X0 + 20, Y0 + 20),
    }
    broken = photo / "B" / "T_BROKEN.JPG"
    broken.write_bytes(b"this is not a jpeg")
    files["broken"] = broken
    (photo / "B" / "video.MP4").write_bytes(b"ignored")
    return project, files


def _engine(base, exiftool_exe, **kw):
    return UAVPhotoOptimizer(RunConfig.create(base_dir=base, exiftool_path=exiftool_exe, **kw))


def test_engine_run_synthetic(standard_project, exiftool_exe):
    base, files = standard_project
    hashes = {k: sha1(p) for k, p in files.items()}
    input_listing = sorted(p.relative_to(base) for p in (base / "input").rglob("*"))

    result = _engine(base, exiftool_exe).run()

    assert isinstance(result, SelectionResult)
    assert result.photos_scanned == 8
    selected = {r.filename for r in result.selected_photos}
    assert selected == {"T01_INSIDE.JPG", "T02_50M.jpg", "T03_99_9M.jpeg",
                        "T06_L2.JPG", "T07_DEEP.jpg"}
    assert [r.filename for r in result.rejected_photos] == ["T04_150M.JPEG"]
    assert [r.filename for r in result.no_gps] == ["T05_NOGPS.JPG"]
    assert [r.filename for r in result.errors] == ["T_BROKEN.JPG"]
    assert result.candidate_photos == result.selected_photos

    assert all(r.reason is PhotoReason.SELECTED for r in result.selected_photos)
    assert result.rejected_photos[0].reason is PhotoReason.OUTSIDE_AOI
    assert result.no_gps[0].reason is PhotoReason.NO_GPS
    assert result.errors[0].reason is PhotoReason.READ_ERROR
    assert "C/C1/C2/子資料夾 中文/T07_DEEP.jpg" in {r.photo_id for r in result.selected_photos}

    assert result.buffer_m == 100 and result.coverage_valid is None
    assert result.selection_mode is CandidateSelectionMode.GPS_POINT
    assert result.aoi.buffer_crs == "EPSG:3826"
    assert any("Phase 6" in w for w in result.warnings)

    # Core never writes: input unchanged, output untouched
    assert sorted(p.relative_to(base) for p in (base / "input").rglob("*")) == input_listing
    assert all(sha1(p) == hashes[k] for k, p in files.items())
    assert not any((base / "output").iterdir())

    # Result is JSON-serialisable for the API
    assert SelectionResult.model_validate_json(result.model_dump_json()) == result


def test_engine_buffer_parameter(standard_project, exiftool_exe):
    base, _ = standard_project
    result = _engine(base, exiftool_exe, buffer_m=0).run()
    assert {r.filename for r in result.selected_photos} == {"T01_INSIDE.JPG", "T06_L2.JPG",
                                                             "T07_DEEP.jpg"}


def test_engine_explicit_shapefile_epsg4326(tmp_path, exiftool_exe):
    from conftest import make_project, write_shp
    base = make_project(tmp_path)
    shp = write_shp(base, crs="EPSG:4326", stem="other")
    write_jpg(base / "input" / "photo" / "IN_99M.JPG", *east_of_square(99))
    write_jpg(base / "input" / "photo" / "OUT_150M.JPG", *east_of_square(150))
    result = _engine(base, exiftool_exe, shapefile=shp).run()
    assert [r.filename for r in result.selected_photos] == ["IN_99M.JPG"]
    assert result.aoi.buffer_crs == "EPSG:32651"


def test_footprint_mode_without_pose_is_unavailable(standard_project, exiftool_exe):
    """Synthetic Pillow JPGs have GPS but no DJI pose → FOOTPRINT_UNAVAILABLE, never a crash."""
    base, _ = standard_project
    result = _engine(base, exiftool_exe,
                     selection_mode=CandidateSelectionMode.FOOTPRINT).run()
    assert not result.selected_photos
    assert {r.reason for r in result.rejected_photos} == {PhotoReason.FOOTPRINT_UNAVAILABLE}
    assert len(result.rejected_photos) == 6


def test_preflight(standard_project, exiftool_exe):
    base, _ = standard_project
    checks = _engine(base, exiftool_exe).preflight()
    assert all(c.ok for c in checks), checks


# Export (consumer side) ---------------------------------------------------------

def test_copy_selected(standard_project, exiftool_exe):
    base, _ = standard_project
    engine = _engine(base, exiftool_exe)
    result = engine.run()
    out = copy_selected(result, base / "output", input_dir=base / "input")
    assert len(out.copied) == 5 and not out.skipped
    # second run never overwrites
    again = copy_selected(result, base / "output", input_dir=base / "input")
    assert not again.copied and len(again.skipped) == 5


def test_copy_duplicate_names_skipped(project, exiftool_exe):
    photo = project / "input" / "photo"
    write_jpg(photo / "A" / "DJI_0001.JPG", X0, Y0)
    write_jpg(photo / "B" / "DJI_0001.JPG", X1, Y1)
    write_jpg(photo / "B" / "DJI_0002.JPG", X1, Y1)
    result = _engine(project, exiftool_exe).run()
    out = copy_selected(result, project / "output")
    assert [p.name for p in out.copied] == ["DJI_0002.JPG"]
    assert len(out.skipped) == 2


def test_copy_into_input_refused(project):
    result = SelectionResult(photos_scanned=0, buffer_m=100, front_overlap_target=80,
                             side_overlap_target=70,
                             selection_mode=CandidateSelectionMode.GPS_POINT)
    with pytest.raises(ExportError):
        copy_selected(result, project / "input" / "photo" / "out", input_dir=project / "input")


def test_cli_new_run_dir_never_reuses(tmp_path):
    from datetime import datetime

    from uav_photo_optimizer.cli import new_run_dir
    root = tmp_path / "output"
    root.mkdir()
    keep = root / "existing.txt"
    keep.write_text("keep")
    t = datetime(2026, 10, 6, 12, 0, 0)
    a, b = new_run_dir(root, t), new_run_dir(root, t)
    assert a.name == "run_20261006_120000" and b.name == "run_20261006_120000_1"
    assert keep.read_text() == "keep"


def test_footprint_mode_end_to_end(project, exiftool_exe):
    """
    Nadir photo 150 m east of the AOI edge at 100 m height: GPS point is outside the 100 m
    buffer, but its 200 m x 200 m estimated footprint (x from +50 to +250 m) reaches into it.
    """
    from conftest import write_dji_jpg
    from uav_photo_optimizer.core.config import FootprintProjectorKind
    from uav_photo_optimizer.footprint.models import FootprintWarning

    photo = project / "input" / "photo"
    write_dji_jpg(photo / "EDGE_150M.JPG", *east_of_square(150))
    write_dji_jpg(photo / "FAR_400M.JPG", *east_of_square(400))
    write_dji_jpg(photo / "INSIDE.JPG", (X0 + X1) / 2, (Y0 + Y1) / 2)
    hashes = {p.name: sha1(p) for p in photo.iterdir()}

    gps = _engine(project, exiftool_exe).run()
    assert {r.filename for r in gps.selected_photos} == {"INSIDE.JPG"}

    from uav_photo_optimizer.metadata.altitude import HeightStrategy
    # synthetic JPGs carry no LRF: AUTO → HEIGHT_UNRESOLVED → candidate only by GPS point
    auto = _engine(project, exiftool_exe, selection_mode=CandidateSelectionMode.FOOTPRINT).run()
    assert {r.filename for r in auto.selected_photos} == {"INSIDE.JPG"}
    assert any("HEIGHT_UNRESOLVED" in w for w in auto.warnings)

    import importlib.util
    kinds = [k for k in FootprintProjectorKind if k is not FootprintProjectorKind.CAMERATRANSFORM
             or importlib.util.find_spec("cameratransform")]       # optional extra
    for kind in kinds:
        fp = _engine(project, exiftool_exe, selection_mode=CandidateSelectionMode.FOOTPRINT,
                     footprint_projector=kind,
                     height_strategy=HeightStrategy.TAKEOFF_RELATIVE).run()
        assert {r.filename for r in fp.selected_photos} == {"INSIDE.JPG", "EDGE_150M.JPG"}
        assert [r.filename for r in fp.rejected_photos] == ["FAR_400M.JPG"]
        rec = next(r for r in fp.selected_photos if r.filename == "EDGE_150M.JPG")
        geom = rec.footprint.geometry
        assert geom.centroid.x == pytest.approx(X1 + 150, abs=0.05)   # GPS dms rounding
        assert geom.area == pytest.approx(200 * 200, rel=1e-6)
        # true-north yaw 0 appears rotated by the grid convergence (~ -0.23° here)
        (x_a, y_a), (x_b, y_b) = geom.exterior.coords[0], geom.exterior.coords[1]
        edge_bearing = math.degrees(math.atan2(x_b - x_a, y_b - y_a)) - 90   # TL→TR edge
        assert edge_bearing == pytest.approx(rec.footprint.provenance["grid_offset_deg"],
                                             abs=1e-6)
        assert -0.3 < rec.footprint.provenance["grid_offset_deg"] < -0.2
        assert FootprintWarning.ALTITUDE_TAKEOFF_RELATIVE in rec.footprint.warnings
        assert FootprintWarning.TERRAIN_NOT_ACCOUNTED_FOR in rec.footprint.warnings
        assert any("Estimated Ground Footprint" in w for w in fp.warnings)
        assert SelectionResult.model_validate_json(fp.model_dump_json()).counts() == fp.counts()

    assert {p.name: sha1(p) for p in photo.iterdir()} == hashes
