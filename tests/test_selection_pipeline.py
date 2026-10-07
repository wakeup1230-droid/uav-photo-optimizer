"""Official v1.0 pipeline: select_photos (six parameters, visual / terrain OFF, COPY)."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from uav_photo_optimizer import AdvancedOptions, SelectionRequest, select_photos
from uav_photo_optimizer.core.exceptions import ConfigError, ExportError
from uav_photo_optimizer.core.selection import ORIGINAL_DATA_NOTE


def test_defaults_are_the_official_parameters(tmp_path):
    r = SelectionRequest(aoi_shapefile=tmp_path / "a.shp", photo_dir=tmp_path)
    assert (r.buffer_m, r.front_overlap, r.side_overlap) == (100, 80, 70)
    with pytest.raises(ValidationError):
        SelectionRequest(aoi_shapefile=tmp_path / "a.shp", photo_dir=tmp_path, front_overlap=60)
    with pytest.raises(ValidationError):
        SelectionRequest(aoi_shapefile=tmp_path / "a.shp", photo_dir=tmp_path, side_overlap=96)
    with pytest.raises(ValidationError):
        SelectionRequest(aoi_shapefile=tmp_path / "a.shp", photo_dir=tmp_path, buffer_m=-1)
    assert AdvancedOptions().visual_guard is False


def test_select_and_copy(dataset):
    base, shp, photo_dir = dataset
    out = base / "result"
    stages = []
    s = select_photos(SelectionRequest(aoi_shapefile=shp, photo_dir=photo_dir, output_dir=out,
                                       buffer_m=0, front_overlap=65, side_overlap=65),
                      progress=lambda st, d, t: stages.append(st), base_dir=base)
    assert s.candidate_photos == s.selected_photos + s.removed_photos
    assert s.removed_photos > 0
    assert s.reduction_percent == round(100 * s.removed_photos / s.candidate_photos, 1)
    run_dir = Path(s.output_dir)
    assert run_dir.parent == out and run_dir.name.startswith("run_")
    copied = sorted(p.name for p in (run_dir / "photos").iterdir())
    assert copied == sorted(i.filename for i in s.selected) and s.copied_photos == len(copied)
    assert not set(copied) & {i.filename for i in s.removed}
    assert (run_dir / "selection_manifest.csv").is_file()
    assert stages[-1] == "done" and "copy" in stages
    # input untouched
    assert all(p.is_file() for p in photo_dir.rglob("*") if p.suffix)


def test_dry_run_without_copy(dataset):
    base, shp, photo_dir = dataset
    s = select_photos(SelectionRequest(aoi_shapefile=shp, photo_dir=photo_dir, buffer_m=0,
                                       front_overlap=65, side_overlap=65, copy_photos=False),
                      base_dir=base)
    assert s.output_dir is None and s.copied_photos == 0 and s.removed_photos > 0
    assert not any((base / "output").iterdir())


def test_original_overlap_below_target_note(dataset):
    base, shp, photo_dir = dataset
    s = select_photos(SelectionRequest(aoi_shapefile=shp, photo_dir=photo_dir, buffer_m=0,
                                       front_overlap=95, side_overlap=95, copy_photos=False),
                      base_dir=base)
    assert s.original_overlap_below_target and s.notes[0] == ORIGINAL_DATA_NOTE


def test_unsafe_requests_refused(dataset):
    base, shp, photo_dir = dataset
    with pytest.raises(ExportError):
        select_photos(SelectionRequest(aoi_shapefile=shp, photo_dir=photo_dir,
                                       output_dir=photo_dir / "out"), base_dir=base)
    with pytest.raises(ConfigError):
        select_photos(SelectionRequest(aoi_shapefile=base / "missing.shp", photo_dir=photo_dir,
                                       output_dir=base / "o"), base_dir=base)
    with pytest.raises(ConfigError):
        select_photos(SelectionRequest(aoi_shapefile=shp, photo_dir=photo_dir), base_dir=base)
