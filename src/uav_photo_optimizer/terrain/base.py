"""
TerrainProvider interface (Part 3).

The Core depends only on this interface — never on weitsicht / rasterio / trimesh classes.

Implementations:

* ``ReferenceRasterRaySolver``        GeoTIFF height field, rasterio + NumPy (independent)
* ``WeitsichtRasterTerrainProvider``  adapter around ``weitsicht.MappingRaster`` (optional)
* ``MeshTerrainProvider``             reserved slot (trimesh), not implemented

Contract:

* coordinates are in ``crs``, a projected CRS with metric X / Y (never lon / lat)
* heights are in ``vertical_datum`` / ``vertical_unit``; callers must align camera heights to
  this datum first (``terrain/alignment.py``)
* ``intersect_rays`` returns the **first** intersection along each ray (height field: one Z
  per XY; overhangs / bridges / wires are not represented)
* thread safety: a provider instance is **not** shared between threads or processes; use
  one provider per job / process (see internal research notes)
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from .models import TerrainIntersectionResult, TerrainSource, VerticalDatumKind


class TerrainProvider(ABC):
    name: str = "abstract"

    crs: str
    vertical_datum: VerticalDatumKind
    vertical_unit: str
    resolution: tuple[float, float]
    nodata: float | None
    source: TerrainSource

    @abstractmethod
    def sample_height(self, x, y) -> np.ndarray:
        """Bilinear height at (x, y) (arrays); NaN outside the raster or near nodata."""

    @abstractmethod
    def intersect_rays(self, origins: np.ndarray, directions: np.ndarray,
                       max_range: np.ndarray | float | None = None) -> TerrainIntersectionResult:
        """First intersection of N rays (origins (N, 3), directions (N, 3) in ``crs``)."""

    @abstractmethod
    def get_bounds(self) -> tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) of the valid sampling area (pixel centres)."""

    def contains_xy(self, x, y) -> np.ndarray:
        xmin, ymin, xmax, ymax = self.get_bounds()
        x, y = np.asarray(x, float), np.asarray(y, float)
        return (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)

    def close(self) -> None:  # noqa: B027  (optional hook)
        """Release file handles."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
