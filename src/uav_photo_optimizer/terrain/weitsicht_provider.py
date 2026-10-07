"""
WeitsichtRasterTerrainProvider (Part 5) — adapter around ``weitsicht.MappingRaster``.

weitsicht is an optional dependency (extra ``terrain``, pinned and tested: 0.0.4, Apache-2.0,
Alpha). Nothing outside this module imports weitsicht; the Core sees only ``TerrainProvider``.

Behaviour of weitsicht 0.0.4 found in the POC (internal research notes):

* the raster CRS must have a Z axis → a compound CRS (e.g. ``EPSG:3826+EPSG:8904``) and the
  same ``crs_s`` for the rays; without a vertical CRS the adapter uses ``force_no_crs``
* ``preload_full_raster=False`` samples the nearest pixel (no interpolation) → up to half a
  pixel × slope height error; the adapter preloads by default
* issues are reported per *batch*, not per ray, and with preload a hit next to a nodata cell
  can be interpolated with the nodata value → every ray is re-checked here:
  - a hit whose bilinear neighbourhood touches nodata → TERRAIN_NODATA
  - a failed ray is classified with the reference solver (OUTSIDE / NODATA /
    NO_INTERSECTION, or NONCONVERGENT when the reference finds a hit)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .base import TerrainProvider
from .models import TerrainIntersectionResult, TerrainIssue, TerrainSource
from .reference import ReferenceRasterRaySolver

TESTED_WEITSICHT = "0.0.4"


def _weitsicht():
    try:
        import weitsicht
        from weitsicht import MappingRaster
    except ImportError as exc:  # pragma: no cover - optional extra
        raise ImportError("WeitsichtRasterTerrainProvider needs weitsicht: pip install "
                          "'uav-photo-optimizer[terrain]'") from exc
    return weitsicht, MappingRaster


class WeitsichtRasterTerrainProvider(TerrainProvider):
    name = "WeitsichtRasterTerrainProvider"

    def __init__(self, source: TerrainSource | str | Path, preload: bool = True):
        import pyproj

        ws, MappingRaster = _weitsicht()
        self.weitsicht_version = getattr(ws, "__version__", None) or _dist_version()
        self.source = (source if isinstance(source, TerrainSource)
                       else TerrainSource(path=Path(source)))
        # own grid + reference solver: bounds, sampling and per-ray issue classification
        self._ref = ReferenceRasterRaySolver(self.source, preload=preload)
        self.grid = self._ref.grid
        self.crs = self.grid.crs
        self.vertical_datum = self.source.vertical_datum
        self.vertical_unit = self.source.vertical_unit
        self.resolution = self._ref.resolution
        self.nodata = self.grid.nodata
        if self.source.vertical_crs and self.crs:
            self._crs = pyproj.CRS(f"{self.crs}+{self.source.vertical_crs}")
            self._mapper = MappingRaster(str(self.source.path), crs=self._crs,
                                         preload_full_raster=preload)
        else:
            self._crs = None
            self._mapper = MappingRaster(str(self.source.path), force_no_crs=True,
                                         preload_full_raster=preload)
        self.preload = preload

    def close(self) -> None:
        self._ref.close()

    def get_bounds(self):
        return self._ref.get_bounds()

    def sample_height(self, x, y) -> np.ndarray:
        return self._ref.sample_height(x, y)

    def intersect_rays(self, origins, directions, max_range=None) -> TerrainIntersectionResult:
        o = np.atleast_2d(np.asarray(origins, float))
        d = np.atleast_2d(np.asarray(directions, float))
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        n = len(o)
        xyz = np.full((n, 3), np.nan)
        valid = np.zeros(n, bool)
        warnings: list[str] = []
        try:
            kw = {"crs_s": self._crs} if self._crs is not None else {}
            r = self._mapper.map_coordinates_from_rays(d, o, **kw)
            if r.ok:
                xyz = np.asarray(r.coordinates, float).copy()
                valid = np.asarray(r.mask, bool).copy() & np.isfinite(xyz).all(axis=1)
            batch_issues = sorted(i.name for i in (r.issues or ()))
        except Exception as exc:  # noqa: BLE001 - adapter boundary
            batch_issues = [f"EXCEPTION:{type(exc).__name__}"]
            warnings.append(f"weitsicht raised {type(exc).__name__}: {exc}")
        if batch_issues:
            warnings.append("weitsicht batch issues: " + ", ".join(batch_issues))
        issues: list = [None] * n
        nodata_hit = np.zeros(n, bool)
        # 1. nodata contamination guard on hits
        if valid.any():
            _, ins, nd = self.grid.sample(xyz[valid, 0], xyz[valid, 1])
            bad = ~ins | nd
            vi = np.flatnonzero(valid)
            for j in np.flatnonzero(bad):
                i = vi[j]
                valid[i] = False
                issues[i] = TerrainIssue.TERRAIN_NODATA if ins[j] else TerrainIssue.TERRAIN_OUTSIDE
                nodata_hit[i] = bool(ins[j])
                xyz[i] = np.nan
            if bad.any():
                warnings.append(f"{int(bad.sum())} weitsicht hits rejected (nodata neighbourhood)")
        # 2. max range
        if max_range is not None and valid.any():
            rng = np.linalg.norm(xyz - o, axis=1)
            far = valid & (rng > np.broadcast_to(np.asarray(max_range, float), (n,)))
            for i in np.flatnonzero(far):
                valid[i], issues[i], xyz[i] = False, TerrainIssue.TERRAIN_NO_INTERSECTION, np.nan
        # 3. classify failed rays with the reference solver
        failed = np.flatnonzero(~valid & np.array([iss is None for iss in issues]))
        if failed.size:
            ref = self._ref.intersect_rays(o[failed], d[failed], None if max_range is None else
                                           np.broadcast_to(np.asarray(max_range, float),
                                                           (n,))[failed])
            for j, i in enumerate(failed):
                if ref.valid[j]:
                    issues[i] = TerrainIssue.TERRAIN_NONCONVERGENT
                else:
                    issues[i] = ref.issues[j]
                    nodata_hit[i] = bool(ref.nodata_hit[j])
        return TerrainIntersectionResult(
            xyz=xyz, valid=valid, provider=self.name,
            method=f"weitsicht {self.weitsicht_version} MappingRaster "
                   f"({'preload' if self.preload else 'disk'})",
            iterations=np.zeros(n, int), inside_bounds=self.contains_xy(o[:, 0], o[:, 1]),
            nodata_hit=nodata_hit, issues=issues, vertical_datum=self.vertical_datum,
            confidence=float(valid.mean()) if n else 0.0, warnings=warnings)


def _dist_version() -> str | None:
    try:
        from importlib.metadata import version
        return version("weitsicht")
    except Exception:  # noqa: BLE001
        return None
