import pytest
from shapely.geometry import Polygon

from conftest import make_project, write_shp
from uav_photo_optimizer.aoi.loader import find_shapefile, load_shapefile
from uav_photo_optimizer.core.exceptions import AOIError


def test_single_shapefile(project):
    assert find_shapefile(project / "input" / "shp").name == "area.shp"


def test_no_shp(tmp_path):
    base = make_project(tmp_path)
    with pytest.raises(AOIError, match="找不到 .shp"):
        find_shapefile(base / "input" / "shp")


def test_two_shp(tmp_path):
    base = make_project(tmp_path)
    write_shp(base, stem="one")
    write_shp(base, stem="two")
    with pytest.raises(AOIError, match="僅允許放置一個 SHP"):
        find_shapefile(base / "input" / "shp")


def test_missing_shp_dir(tmp_path):
    with pytest.raises(AOIError, match="SHP 資料夾"):
        find_shapefile(tmp_path / "nope")


def test_missing_prj(project):
    shp = project / "input" / "shp" / "area.shp"
    shp.with_suffix(".prj").unlink()
    with pytest.raises(AOIError, match=r"\.prj"):
        load_shapefile(shp)


def test_missing_shx(project):
    shp = project / "input" / "shp" / "area.shp"
    shp.with_suffix(".shx").unlink()
    with pytest.raises(AOIError, match=r"\.shx"):
        load_shapefile(shp)


def test_load_ok(project):
    gdf = load_shapefile(project / "input" / "shp" / "area.shp")
    assert len(gdf) == 1 and gdf.crs.to_epsg() == 3826


def test_invalid_geometry_repaired_with_warning(tmp_path):
    base = make_project(tmp_path)
    bowtie = Polygon([(305500, 2655000), (305700, 2655200), (305700, 2655000),
                      (305500, 2655200)])
    write_shp(base, geom=bowtie)
    warnings = []
    gdf = load_shapefile(base / "input" / "shp" / "area.shp", warnings)
    assert gdf.geometry.is_valid.all()
    assert any("make_valid" in w for w in warnings)


def test_shapefile_not_modified(project):
    from conftest import sha1
    shp_dir = project / "input" / "shp"
    before = {p.name: sha1(p) for p in shp_dir.iterdir()}
    load_shapefile(shp_dir / "area.shp")
    assert {p.name: sha1(p) for p in shp_dir.iterdir()} == before
