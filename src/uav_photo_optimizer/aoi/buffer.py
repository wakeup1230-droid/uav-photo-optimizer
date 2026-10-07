"""Buffered AOI geometry."""

from __future__ import annotations

from dataclasses import dataclass

import geopandas as gpd
import shapely
from pyproj import CRS
from shapely.geometry.base import BaseGeometry

from ..core.config import BUFFER_QUAD_SEGS
from ..core.exceptions import AOIError
from .crs import choose_metric_crs


@dataclass(frozen=True)
class AOIBuffer:
    geometry: BaseGeometry      # prepared, in ``crs``
    crs: CRS                    # metre-based CRS the buffer was computed in
    source_crs: CRS
    buffer_m: float


def build_buffer(gdf: gpd.GeoDataFrame, buffer_m: float,
                 quad_segs: int = BUFFER_QUAD_SEGS) -> AOIBuffer:
    """Union all AOI geometries in a metre CRS and buffer by ``buffer_m`` metres."""
    if buffer_m < 0:
        raise AOIError(f"buffer_m 不可小於 0：{buffer_m}")
    metric_crs = choose_metric_crs(gdf)
    geometry = gdf.to_crs(metric_crs).geometry.union_all()
    buffer_geom = geometry.buffer(buffer_m, quad_segs=quad_segs)
    if buffer_geom.is_empty:
        raise AOIError("Buffer 結果為空，請檢查 SHP Geometry")
    shapely.prepare(buffer_geom)
    return AOIBuffer(geometry=buffer_geom, crs=CRS(metric_crs),
                     source_crs=CRS(gdf.crs), buffer_m=buffer_m)
