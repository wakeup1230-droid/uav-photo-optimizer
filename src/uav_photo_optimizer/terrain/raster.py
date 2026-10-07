"""
GeoTIFF height-field access (rasterio + NumPy), shared by the terrain providers.

* pixel-centre convention: the value of pixel (row, col) belongs to its centre
* bilinear interpolation between the four surrounding centres; if any of them is nodata the
  sample is NaN and flagged as nodata (no nodata contamination)
* the valid sampling area is the hull of the pixel centres (half a pixel inside the edges)
* disk-backed mode reads only the window a query needs; preload reads the band once
* rasterio dataset handles are not thread-safe: reads go through a lock, and a provider is
  meant to be used by one job / process (open a new provider in each worker process)
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _rasterio():
    try:
        import rasterio
    except ImportError as exc:  # pragma: no cover - depends on optional extra
        raise ImportError("terrain support needs rasterio: pip install "
                          "'uav-photo-optimizer[terrain]'") from exc
    return rasterio


@dataclass
class _Window:
    data: np.ndarray            # float64, NaN = nodata
    row0: int
    col0: int


class RasterGrid:
    """Single-band north-up GeoTIFF height field."""

    def __init__(self, path: str | Path, preload: bool = True, band: int = 1):
        rio = _rasterio()
        self.path = Path(path)
        self._ds = rio.open(self.path)
        self._lock = threading.Lock()
        ds = self._ds
        t = ds.transform
        if t.b != 0 or t.d != 0 or t.e >= 0:
            raise ValueError("raster must be north-up without rotation")
        self.band = band
        self.width, self.height = ds.width, ds.height
        self.x0, self.y0 = t.c, t.f
        self.rx, self.ry = t.a, -t.e
        self.nodata = ds.nodata
        self.crs = ds.crs.to_string() if ds.crs else None
        self.count = ds.count
        self.dtype = ds.dtypes[band - 1]
        self.preloaded = preload
        self._full: _Window | None = None
        self.reads = 0
        if preload:
            self._full = self._read(0, 0, self.height, self.width)
            valid = self._full.data[np.isfinite(self._full.data)]
        else:
            valid = self._block_stats()
        self.zmin = float(valid.min()) if valid.size else math.nan
        self.zmax = float(valid.max()) if valid.size else math.nan
        self.valid_count = int(valid.size) if preload else self._valid_count
        self.nodata_fraction = 1.0 - self.valid_count / float(self.width * self.height)

    # -- io -----------------------------------------------------------------------------

    def _read(self, row0: int, col0: int, rows: int, cols: int) -> _Window:
        from rasterio.windows import Window
        with self._lock:
            a = self._ds.read(self.band, window=Window(col0, row0, cols, rows)).astype(float)
            self.reads += 1
        if self.nodata is not None and not math.isnan(self.nodata):
            a[a == self.nodata] = np.nan
        return _Window(a, row0, col0)

    def _block_stats(self) -> np.ndarray:
        lo, hi, n = math.inf, -math.inf, 0
        for _, win in self._ds.block_windows(self.band):
            w = self._read(int(win.row_off), int(win.col_off), int(win.height), int(win.width))
            v = w.data[np.isfinite(w.data)]
            if v.size:
                lo, hi, n = min(lo, v.min()), max(hi, v.max()), n + v.size
        self._valid_count = n
        return np.array([lo, hi]) if n else np.array([])

    def close(self) -> None:
        self._ds.close()

    # -- geometry -----------------------------------------------------------------------

    def bounds_centres(self) -> tuple[float, float, float, float]:
        return (self.x0 + 0.5 * self.rx, self.y0 - (self.height - 0.5) * self.ry,
                self.x0 + (self.width - 0.5) * self.rx, self.y0 - 0.5 * self.ry)

    def bounds_edges(self) -> tuple[float, float, float, float]:
        return (self.x0, self.y0 - self.height * self.ry,
                self.x0 + self.width * self.rx, self.y0)

    def fractional(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        """Fractional (col, row) of pixel centres: integer values sit on centres."""
        return ((np.asarray(x, float) - self.x0) / self.rx - 0.5,
                (self.y0 - np.asarray(y, float)) / self.ry - 0.5)

    def window_for(self, x, y, pad: int = 2) -> _Window:
        if self._full is not None:
            return self._full
        fc, fr = self.fractional(x, y)
        ok = np.isfinite(fc) & np.isfinite(fr)
        if not ok.any():
            return _Window(np.full((1, 1), np.nan), 0, 0)
        c0 = int(max(0, math.floor(np.nanmin(fc[ok])) - pad))
        c1 = int(min(self.width, math.floor(np.nanmax(fc[ok])) + pad + 2))
        r0 = int(max(0, math.floor(np.nanmin(fr[ok])) - pad))
        r1 = int(min(self.height, math.floor(np.nanmax(fr[ok])) + pad + 2))
        if c1 <= c0 or r1 <= r0:
            return _Window(np.full((1, 1), np.nan), 0, 0)
        return self._read(r0, c0, r1 - r0, c1 - c0)

    # -- sampling -----------------------------------------------------------------------

    def sample(self, x, y, window: _Window | None = None
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(z, inside, nodata) for arrays x, y. z is NaN when not inside or nodata."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        w = window if window is not None else self.window_for(x.ravel(), y.ravel())
        fc, fr = self.fractional(x, y)
        inside = ((fc >= 0) & (fc <= self.width - 1) & (fr >= 0) & (fr <= self.height - 1))
        z = np.full(x.shape, np.nan)
        nodata = np.zeros(x.shape, bool)
        if not inside.any():
            return z, inside, nodata
        c = fc[inside] - w.col0
        r = fr[inside] - w.row0
        h, wd = w.data.shape
        c0 = np.clip(np.floor(c).astype(int), 0, max(wd - 2, 0))
        r0 = np.clip(np.floor(r).astype(int), 0, max(h - 2, 0))
        c1 = np.minimum(c0 + 1, wd - 1)
        r1 = np.minimum(r0 + 1, h - 1)
        in_win = (c >= -1e-9) & (c <= wd - 1 + 1e-9) & (r >= -1e-9) & (r <= h - 1 + 1e-9)
        fx = np.clip(c - c0, 0, 1)
        fy = np.clip(r - r0, 0, 1)
        d = w.data
        v00, v01, v10, v11 = d[r0, c0], d[r0, c1], d[r1, c0], d[r1, c1]
        val = (v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy)
               + v10 * (1 - fx) * fy + v11 * fx * fy)
        bad = ~np.isfinite(val) | ~in_win
        val[bad] = np.nan
        z[inside] = val
        nd = np.zeros(int(inside.sum()), bool)
        nd[bad] = True
        nodata[inside] = nd
        return z, inside, nodata
