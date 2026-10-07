"""
Terrain preflight (Parts 9–12): may this raster be used for terrain footprints?

Hard checks (any failure → NOT_READY):

* GeoTIFF, readable, single band, north-up
* CRS present, projected, X / Y in metres (no ray intersection in lon / lat)
* resolution > 0 and plausible (≤ 100 m)
* nodata declared
* vertical unit metre; a declared vertical CRS must be a vertical CRS
* AOI covered by valid cells (≥ ``min_aoi_coverage``)
* camera rays covered: planar footprints inside the raster (≥ ``min_ray_coverage``)

The vertical datum gate itself is decided by ``terrain/alignment.py``.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry

from .models import TerrainCheck, TerrainDiagnostics, TerrainSource, TerrainStatus

MAX_RESOLUTION_M = 100.0


def _sample_points(geom: BaseGeometry, spacing: float, limit: int = 20000) -> np.ndarray:
    import shapely
    xmin, ymin, xmax, ymax = geom.bounds
    area = max(geom.area, 1e-9)
    spacing = max(spacing, math.sqrt(area / limit))
    xs = np.arange(xmin + spacing / 2, xmax, spacing)
    ys = np.arange(ymin + spacing / 2, ymax, spacing)
    if xs.size == 0 or ys.size == 0:
        c = geom.representative_point()
        return np.array([[c.x, c.y]])
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    return pts[shapely.contains_xy(geom, pts[:, 0], pts[:, 1])]


def terrain_preflight(source: TerrainSource, aoi: Optional[BaseGeometry] = None,
                      aoi_crs: Optional[str] = None,
                      footprints: Optional[Iterable[BaseGeometry]] = None,
                      min_aoi_coverage: float = 0.95,
                      min_ray_coverage: float = 0.95) -> TerrainDiagnostics:
    checks: list[TerrainCheck] = []
    diag = TerrainDiagnostics(status=TerrainStatus.NOT_READY,
                              vertical_datum=source.vertical_datum)

    def add(name: str, ok: bool, detail: str = "") -> bool:
        checks.append(TerrainCheck(name=name, ok=bool(ok), detail=detail))
        return ok

    try:
        import rasterio
        from pyproj import CRS
    except ImportError as exc:
        add("rasterio available", False, str(exc))
        diag.checks = checks
        diag.reasons = [c.name for c in checks if not c.ok]
        return diag

    if not add("file exists", source.path.is_file(), str(source.path.name)):
        diag.checks, diag.reasons = checks, ["file exists"]
        return diag
    try:
        ds = rasterio.open(source.path)
    except Exception as exc:  # noqa: BLE001
        add("readable", False, f"{type(exc).__name__}: {exc}")
        diag.checks, diag.reasons = checks, ["readable"]
        return diag
    with ds:
        add("readable", True)
        add("GeoTIFF", ds.driver == "GTiff", ds.driver)
        add("single band", ds.count == 1, f"{ds.count} bands")
        t = ds.transform
        add("north-up, no rotation", t.b == 0 and t.d == 0 and t.e < 0)
        crs = CRS.from_wkt(ds.crs.to_wkt()) if ds.crs else None
        add("CRS present", crs is not None, crs.to_string() if crs else "missing")
        if crs is not None:
            horiz = crs.sub_crs_list[0] if crs.is_compound else crs
            unit = horiz.axis_info[0].unit_name if horiz.axis_info else ""
            add("projected CRS (no lon/lat)", horiz.is_projected, horiz.name)
            add("X/Y unit metre", unit in ("metre", "meter", "m"), unit)
            diag.crs = horiz.to_string()
            embedded_v = crs.sub_crs_list[1] if crs.is_compound else None
            if embedded_v is not None and source.vertical_crs:
                same = CRS(source.vertical_crs).equals(embedded_v)
                add("declared vertical CRS matches raster", same,
                    f"{source.vertical_crs} vs {embedded_v.name}")
        rx, ry = t.a, -t.e
        diag.resolution = (rx, ry)
        add("resolution plausible", 0 < rx <= MAX_RESOLUTION_M and 0 < ry <= MAX_RESOLUTION_M,
            f"{rx:g} × {ry:g}")
        diag.nodata = ds.nodata
        add("nodata declared", ds.nodata is not None, str(ds.nodata))
        add("vertical unit metre", source.vertical_unit in ("metre", "meter", "m"),
            source.vertical_unit)
        if source.vertical_crs:
            try:
                vok = CRS(source.vertical_crs).is_vertical
            except Exception:  # noqa: BLE001
                vok = False
            add("vertical CRS is vertical", vok, source.vertical_crs)
        bounds = ds.bounds
        diag.bounds = (bounds.left, bounds.bottom, bounds.right, bounds.top)
        rbox = box(*diag.bounds)

        if aoi is not None:
            g = aoi
            if aoi_crs and diag.crs and not CRS(aoi_crs).equals(CRS(diag.crs)):
                add("AOI CRS equals raster CRS", False, f"{aoi_crs} vs {diag.crs}")
            else:
                pts = _sample_points(g, max(rx, ry) * 4)
                vals = _sample_valid(ds, pts)
                cov = float(vals.mean()) if vals.size else 0.0
                diag.aoi_coverage = round(cov, 4)
                add("AOI covered by valid cells", cov >= min_aoi_coverage,
                    f"{cov:.1%} of {vals.size} samples")
        if footprints is not None:
            fps = list(footprints)
            if fps:
                inside = sum(1 for f in fps if rbox.contains(f))
                frac = inside / len(fps)
                add("camera rays covered (planar footprints inside raster)",
                    frac >= min_ray_coverage, f"{inside}/{len(fps)} = {frac:.1%}")
        # nodata fraction (cheap: decimated read)
        try:
            step = max(1, int(max(ds.width, ds.height) / 2000))
            a = ds.read(1, out_shape=(max(1, ds.height // step), max(1, ds.width // step)))
            nd = ~np.isfinite(a.astype(float))
            if ds.nodata is not None:
                nd |= a == ds.nodata
            diag.nodata_fraction = round(float(nd.mean()), 4)
        except Exception:  # noqa: BLE001
            pass
    diag.checks = checks
    diag.reasons = [c.name + (f" ({c.detail})" if c.detail else "") for c in checks if not c.ok]
    diag.status = TerrainStatus.READY if all(c.ok for c in checks) else TerrainStatus.NOT_READY
    return diag


def _sample_valid(ds, pts: np.ndarray) -> np.ndarray:
    if pts.size == 0:
        return np.zeros(0, bool)
    vals = np.array([v[0] for v in ds.sample([tuple(p) for p in pts], indexes=1)], float)
    ok = np.isfinite(vals)
    if ds.nodata is not None:
        ok &= vals != ds.nodata
    l, b, r, t = ds.bounds
    ok &= (pts[:, 0] > l) & (pts[:, 0] < r) & (pts[:, 1] > b) & (pts[:, 1] < t)
    return ok
