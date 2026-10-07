import math

import geopandas as gpd
import pytest
from shapely.affinity import translate
from shapely.geometry import Point

from conftest import SQUARE, X0, X1, Y0, Y1
from uav_photo_optimizer.aoi.buffer import build_buffer
from uav_photo_optimizer.core.exceptions import AOIError


@pytest.fixture
def gdf():
    return gpd.GeoDataFrame({"id": [1]}, geometry=[SQUARE], crs="EPSG:3826")


def test_bounds_and_area(gdf):
    buf = build_buffer(gdf, 100)
    assert buf.crs.to_epsg() == 3826
    assert buf.geometry.bounds == pytest.approx((X0 - 100, Y0 - 100, X1 + 100, Y1 + 100))
    expected = 200 * 200 + 4 * 200 * 100 + math.pi * 100 ** 2
    assert buf.geometry.area == pytest.approx(expected, rel=1e-3)


def test_boundary_point_is_covered(gdf):
    """A point exactly on the buffer boundary counts as inside (covers, not contains)."""
    buf = build_buffer(gdf, 100)
    on_edge = Point(X1 + 100, (Y0 + Y1) / 2)
    assert buf.geometry.boundary.distance(on_edge) < 1e-6
    assert buf.geometry.covers(on_edge)
    assert not buf.geometry.contains(on_edge)


def test_zero_buffer_equals_aoi(gdf):
    assert build_buffer(gdf, 0).geometry.equals(SQUARE)


def test_negative_buffer_rejected(gdf):
    with pytest.raises(AOIError):
        build_buffer(gdf, -1)


def test_multiple_features_are_unioned():
    far = translate(SQUARE, xoff=1000)          # second square 1 km east
    gdf = gpd.GeoDataFrame({"id": [1, 2]}, geometry=[SQUARE, far], crs="EPSG:3826")
    buf = build_buffer(gdf, 10)
    assert buf.geometry.geom_type == "MultiPolygon"
