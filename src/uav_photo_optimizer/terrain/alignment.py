"""
TerrainVerticalAlignment (Parts 12–15): camera heights in the terrain's vertical datum.

Never subtract an unknown-datum DJI AbsoluteAltitude from a DEM height directly.

Modes:

LRF_ANCHORED (AUTO)
    For every photo with a VALID laser (Height Strategy v2, LRF_RAY_VERTICAL):
        Z_cam = DEM(laser target XY) + laser vertical height
    The datum of AbsoluteAltitude cancels out. Offsets ``Z_cam − AbsoluteAltitude`` of the
    anchors give a per-flight datum offset (median); photos without a valid laser get
    ``Z_cam = AbsoluteAltitude + offset(flight)``. The robust spread of the offsets is the
    alignment quality; too few anchors or too large a spread → NOT_READY.
EXPLICIT_VERTICAL_TRANSFORM
    The user supplies a ``VerticalTransform`` (constant offset, or a geoid / GTG grid through
    ``GridVerticalTransform`` with a user-supplied file). The DEM datum must be declared.
    No national grid is hard-coded in Core.
ASSUME_SAME_DATUM
    Z_cam = AbsoluteAltitude. Only when ``assume_same_datum_confirmed`` is set by the user;
    never chosen automatically.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections import defaultdict
from pathlib import Path
from typing import Optional

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from ..metadata.models import PhotoMetadata
from .base import TerrainProvider
from .models import TerrainConfig, TerrainStatus, VerticalAlignmentMode, VerticalDatumKind


class VerticalTransform(ABC):
    """Photo altitude (its own datum) → terrain vertical datum, at grid (x, y)."""

    description: str = ""

    @abstractmethod
    def to_terrain(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        ...


class ConstantOffsetTransform(VerticalTransform):
    def __init__(self, offset_m: float, description: str = "constant offset"):
        self.offset_m = float(offset_m)
        self.description = f"{description}: {offset_m:+.3f} m"

    def to_terrain(self, x, y, z):
        return np.asarray(z, float) + self.offset_m


class GridVerticalTransform(VerticalTransform):
    """
    ``Z_terrain = z − N(x, y)`` with N read bilinearly from a user-supplied single-band grid
    (e.g. a geoid undulation / GTG GeoTIFF in the terrain CRS). The grid is never bundled.
    """

    def __init__(self, grid_path: str | Path, sign: float = -1.0, description: str = ""):
        from .raster import RasterGrid
        self.grid = RasterGrid(grid_path, preload=True)
        self.sign = sign
        self.description = description or f"grid {Path(grid_path).name} (sign {sign:+g})"

    def to_terrain(self, x, y, z):
        n = self.grid.sample(np.asarray(x, float), np.asarray(y, float))[0]
        return np.asarray(z, float) + self.sign * n


class AlignedHeight(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_id: str
    camera_z: float
    method: str                          # LRF_ANCHORED / FLIGHT_OFFSET / EXPLICIT / SAME_DATUM
    offset_m: Optional[float] = None     # camera_z − AbsoluteAltitude


class TerrainVerticalAlignment(BaseModel):
    model_config = ConfigDict(extra="forbid")

    status: TerrainStatus
    mode: VerticalAlignmentMode
    terrain_datum: VerticalDatumKind
    heights: dict[str, AlignedHeight] = Field(default_factory=dict)
    anchors: int = 0
    anchors_rejected: int = 0
    offset_median_m: Optional[float] = None
    offset_spread_m: Optional[float] = None          # 1.4826 × MAD
    offset_p95_abs_dev_m: Optional[float] = None
    flight_offsets_m: dict[str, float] = Field(default_factory=dict)
    reasons: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)

    def summary(self) -> dict:
        return self.model_dump(exclude={"heights"})


def _not_ready(mode, datum, *reasons) -> TerrainVerticalAlignment:
    return TerrainVerticalAlignment(status=TerrainStatus.NOT_READY, mode=mode,
                                    terrain_datum=datum, reasons=list(reasons))


def align_vertical(provider: TerrainProvider, config: TerrainConfig,
                   metadata: dict[str, PhotoMetadata], footprints: dict,
                   flights: Optional[dict[str, str]] = None,
                   transform: Optional[VerticalTransform] = None,
                   target_crs: Optional[str] = None) -> TerrainVerticalAlignment:
    """
    ``footprints``: planar PhotoFootprints (Height Strategy v2) — their LRF quality and laser
    height are the anchors. ``flights``: photo_id → flight_id.
    """
    mode = config.alignment
    datum = provider.vertical_datum
    if mode is VerticalAlignmentMode.AUTO:
        mode = VerticalAlignmentMode.LRF_ANCHORED

    if mode is VerticalAlignmentMode.ASSUME_SAME_DATUM:
        if not config.assume_same_datum_confirmed:
            return _not_ready(mode, datum, "ASSUME_SAME_DATUM requires explicit user "
                              "confirmation (assume_same_datum_confirmed=True)")
        out = TerrainVerticalAlignment(status=TerrainStatus.READY, mode=mode, terrain_datum=datum,
                                       warnings=["user asserted: photo altitude and terrain "
                                                 "share one vertical datum (unverified)"])
        for pid, m in metadata.items():
            if m.absolute_altitude is not None:
                out.heights[pid] = AlignedHeight(photo_id=pid, camera_z=m.absolute_altitude,
                                                 method="SAME_DATUM", offset_m=0.0)
        return out

    if mode is VerticalAlignmentMode.EXPLICIT_VERTICAL_TRANSFORM:
        if datum is VerticalDatumKind.UNKNOWN:
            return _not_ready(mode, datum, "terrain vertical datum UNKNOWN: an explicit "
                              "transform needs a declared terrain datum")
        if transform is None and config.explicit_offset_m is not None:
            transform = ConstantOffsetTransform(config.explicit_offset_m, "explicit offset")
        if transform is None:
            return _not_ready(mode, datum, "no vertical transform supplied")
        from ..footprint.pose import wgs84_to
        to_grid = wgs84_to(target_crs or provider.crs)
        out = TerrainVerticalAlignment(status=TerrainStatus.READY, mode=mode, terrain_datum=datum,
                                       warnings=[transform.description])
        ids = [p for p, m in metadata.items() if m.absolute_altitude is not None and m.has_gps]
        if ids:
            xy = np.array([to_grid.transform(metadata[p].longitude, metadata[p].latitude)
                           for p in ids])
            z = transform.to_terrain(xy[:, 0], xy[:, 1],
                                     np.array([metadata[p].absolute_altitude for p in ids]))
            for p, zz in zip(ids, z):
                if math.isfinite(zz):
                    out.heights[p] = AlignedHeight(
                        photo_id=p, camera_z=float(zz), method="EXPLICIT",
                        offset_m=float(zz - metadata[p].absolute_altitude))
        return out

    # LRF_ANCHORED
    from ..footprint.lrf import LRFQualityStatus
    from ..footprint.pose import wgs84_to
    to_grid = wgs84_to(target_crs or provider.crs)
    anchor_ids, txy, lh = [], [], []
    for pid, fp in footprints.items():
        m = metadata.get(pid)
        q = fp.lrf_quality
        if (m is None or q is None or q.status is not LRFQualityStatus.VALID
                or fp.height_strategy != "LRF_RAY_VERTICAL" or m.absolute_altitude is None
                or m.lrf_target_lat is None or m.lrf_target_lon is None
                or q.laser_vertical_height_m is None):
            continue
        anchor_ids.append(pid)
        txy.append(to_grid.transform(m.lrf_target_lon, m.lrf_target_lat))
        lh.append(q.laser_vertical_height_m)
    if not anchor_ids:
        return _not_ready(mode, datum, "no VALID LRF anchor: vertical datum cannot be "
                          "bridged (Terrain Mode NOT_READY)")
    txy = np.array(txy)
    zdem = provider.sample_height(txy[:, 0], txy[:, 1])
    zcam = zdem + np.array(lh)
    absalt = np.array([metadata[p].absolute_altitude for p in anchor_ids])
    off = zcam - absalt
    ok = np.isfinite(off)
    out = TerrainVerticalAlignment(status=TerrainStatus.NOT_READY, mode=mode, terrain_datum=datum,
                                   anchors=int(ok.sum()), anchors_rejected=int((~ok).sum()))
    if ok.sum() < config.min_lrf_anchors:
        out.reasons.append(f"only {int(ok.sum())} LRF anchors on valid terrain "
                           f"(< {config.min_lrf_anchors})")
        return out
    med = float(np.median(off[ok]))
    per_flight = defaultdict(list)
    for p, o_, good in zip(anchor_ids, off, ok):
        if good:
            per_flight[(flights or {}).get(p, "")].append(o_)
    out.flight_offsets_m = {k: round(float(np.median(v)), 3) for k, v in sorted(per_flight.items())}
    # quality = spread of each anchor around its own flight's offset (the datum offset may
    # legitimately differ between flights, e.g. another RTK base / take-off session)
    dev = np.array([abs(o_ - out.flight_offsets_m[(flights or {}).get(p, "")])
                    for p, o_, good in zip(anchor_ids, off, ok) if good])
    spread = float(1.4826 * np.median(dev))
    out.offset_median_m = round(med, 3)
    out.offset_spread_m = round(spread, 3)
    out.offset_p95_abs_dev_m = round(float(np.percentile(dev, 95)), 3)
    fo = list(out.flight_offsets_m.values())
    if len(fo) > 1 and max(fo) - min(fo) > config.max_anchor_offset_spread_m:
        out.warnings.append(f"datum offset differs between flights by "
                            f"{max(fo) - min(fo):.2f} m (per-flight offsets applied)")
    if spread > config.max_anchor_offset_spread_m:
        out.reasons.append(f"anchor offset spread {spread:.2f} m > "
                           f"{config.max_anchor_offset_spread_m:g} m: terrain and LRF disagree")
        return out
    if datum is VerticalDatumKind.UNKNOWN:
        out.warnings.append("terrain vertical datum UNKNOWN: camera heights are tied to the "
                            "terrain by LRF anchoring only (datum-independent)")
    out.status = TerrainStatus.READY
    anchored = {p: (z, o_) for p, z, o_, good in zip(anchor_ids, zcam, off, ok) if good}
    for pid, m in metadata.items():
        if pid in anchored:
            z, o_ = anchored[pid]
            out.heights[pid] = AlignedHeight(photo_id=pid, camera_z=float(z),
                                             method="LRF_ANCHORED", offset_m=float(o_))
        elif m.absolute_altitude is not None:
            fo = out.flight_offsets_m.get((flights or {}).get(pid, ""), med)
            out.heights[pid] = AlignedHeight(photo_id=pid, camera_z=m.absolute_altitude + fo,
                                             method="FLIGHT_OFFSET", offset_m=fo)
    return out
