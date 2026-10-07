import math

import geopandas as gpd
import pytest

from conftest import SQUARE
from uav_photo_optimizer.aoi.buffer import build_buffer
from uav_photo_optimizer.aoi.crs import choose_metric_crs


def _gdf(crs):
    return gpd.GeoDataFrame({"id": [1]}, geometry=[SQUARE], crs="EPSG:3826").to_crs(crs)


def test_projected_metre_crs_kept():
    assert choose_metric_crs(_gdf("EPSG:3826")).to_epsg() == 3826


def test_geographic_crs_goes_to_utm():
    crs = choose_metric_crs(_gdf("EPSG:4326"))
    assert crs.is_projected and crs.to_epsg() == 32651     # UTM 51N for Taiwan


def test_non_metre_projected_goes_to_utm():
    # EPSG:2227 = NAD83 / California zone 3 (US survey feet)
    crs = choose_metric_crs(_gdf("EPSG:2227"))
    assert crs.axis_info[0].unit_name.lower() in ("metre", "meter")


def test_epsg4326_buffer_is_metres_not_degrees():
    buf = build_buffer(_gdf("EPSG:4326"), 100)
    expected = 200 * 200 + 4 * 200 * 100 + math.pi * 100 ** 2
    assert buf.geometry.area == pytest.approx(expected, rel=0.005)
    assert buf.source_crs.to_epsg() == 4326
