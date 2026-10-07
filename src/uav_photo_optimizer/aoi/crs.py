"""CRS helpers."""

from __future__ import annotations

import geopandas as gpd
from pyproj import CRS

from ..core.exceptions import AOIError

PHOTO_CRS = "EPSG:4326"     # photo GPS is treated as WGS84


def choose_metric_crs(gdf: gpd.GeoDataFrame) -> CRS:
    """
    Return a metre-based CRS suitable for buffering.

    Projected metre CRS (e.g. EPSG:3826) is used as is; geographic or non-metre
    projected CRS → UTM zone estimated from the AOI location.
    """
    crs = gdf.crs
    if crs.is_projected:
        unit = crs.axis_info[0].unit_name.lower() if crs.axis_info else ""
        if unit in ("metre", "meter"):
            return crs
    utm_crs = gdf.estimate_utm_crs()
    if utm_crs is None:
        raise AOIError("無法依 SHP 位置自動選擇公尺制投影座標系統")
    return utm_crs


def crs_label(crs) -> str:
    return CRS(crs).to_string()
