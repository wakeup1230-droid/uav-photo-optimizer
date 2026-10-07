"""
ReferenceRasterRaySolver (Part 8) — independent first-hit ray / height-field intersection.

Written from scratch on rasterio + NumPy (no code taken from weitsicht). Algorithm:

1. skip the part of the ray above the raster maximum (downward rays)
2. march along the ray with a step of ``step_fraction × pixel`` in the dominant axis
   (horizontal for oblique rays, vertical for steep rays), sampling the bilinear surface
3. the first step where ray height ≤ surface height brackets the **first** hit; any nodata
   sample or exit from the raster before that is reported instead (TERRAIN_NODATA /
   TERRAIN_OUTSIDE), so a hit is never reported behind a hole
4. bisection on the bracket until |Δz| < ``tolerance_m``

Limitation (documented): a surface feature thinner than one step along the ray can be
skipped (a sliver below half a pixel); the bilinear surface itself is the model.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .base import TerrainProvider
from .models import TerrainIntersectionResult, TerrainIssue, TerrainSource
from .raster import RasterGrid

STEP_BLOCK = 256
MAX_BISECTION = 60


class ReferenceRasterRaySolver(TerrainProvider):
    name = "ReferenceRasterRaySolver"

    def __init__(self, source: TerrainSource | str | Path, preload: bool = True,
                 step_fraction: float = 0.5, tolerance_m: float = 0.001):
        self.source = (source if isinstance(source, TerrainSource)
                       else TerrainSource(path=Path(source)))
        self.grid = RasterGrid(self.source.path, preload=preload)
        self.crs = self.grid.crs
        self.vertical_datum = self.source.vertical_datum
        self.vertical_unit = self.source.vertical_unit
        self.resolution = (self.grid.rx, self.grid.ry)
        self.nodata = self.grid.nodata
        self.step_fraction = step_fraction
        self.tolerance_m = tolerance_m

    def close(self) -> None:
        self.grid.close()

    def get_bounds(self):
        return self.grid.bounds_centres()

    def sample_height(self, x, y) -> np.ndarray:
        return self.grid.sample(x, y)[0]

    # -- intersection -------------------------------------------------------------------

    def intersect_rays(self, origins, directions, max_range=None) -> TerrainIntersectionResult:
        o = np.atleast_2d(np.asarray(origins, float))
        d = np.atleast_2d(np.asarray(directions, float))
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        n = len(o)
        g = self.grid
        xyz = np.full((n, 3), np.nan)
        valid = np.zeros(n, bool)
        iters = np.zeros(n, int)
        nodata_hit = np.zeros(n, bool)
        issues: list = [None] * n
        warnings: list[str] = []
        inside0 = self.contains_xy(o[:, 0], o[:, 1])
        if not math.isfinite(g.zmax):
            return TerrainIntersectionResult(
                xyz=xyz, valid=valid, provider=self.name, method="RAY_MARCH_BISECTION",
                iterations=iters, inside_bounds=inside0, nodata_hit=nodata_hit,
                issues=[TerrainIssue.TERRAIN_NODATA] * n, vertical_datum=self.vertical_datum,
                confidence=0.0, warnings=["raster has no valid cell"])

        xmin, ymin, xmax, ymax = g.bounds_edges()
        diag = math.hypot(xmax - xmin, ymax - ymin)
        step = self.step_fraction * min(g.rx, g.ry)
        hcomp = np.hypot(d[:, 0], d[:, 1])
        dt = step / np.maximum(np.maximum(hcomp, np.abs(d[:, 2])), 1e-9)
        # search range
        if max_range is None:
            t_end = np.full(n, np.inf)
        else:
            t_end = np.broadcast_to(np.asarray(max_range, float), (n,)).copy()
        down = d[:, 2] < -1e-12
        with np.errstate(divide="ignore", invalid="ignore"):
            t_below = np.where(down, (o[:, 2] - (g.zmin - 1.0)) / -d[:, 2], np.inf)
            t_above = np.where(down, np.maximum(0.0, (o[:, 2] - g.zmax) / -d[:, 2]), 0.0)
        horiz_limit = np.where(hcomp > 1e-12,
                               (diag + np.hypot(o[:, 0] - (xmin + xmax) / 2,
                                                o[:, 1] - (ymin + ymax) / 2)) / np.maximum(hcomp, 1e-12),
                               np.inf)
        t_end = np.minimum(np.minimum(t_end, t_below), horiz_limit)
        t_end = np.where(np.isfinite(t_end), t_end, 0.0)
        # upward / horizontal ray already above the whole surface: no intersection
        never = (~down) & (o[:, 2] > g.zmax)
        t_start = np.where(o[:, 2] > g.zmax, t_above, 0.0)
        # ray starting below the surface → misalignment, not a hit
        z0, in0, nd0 = g.sample(o[:, 0], o[:, 1])
        below0 = in0 & ~nd0 & (o[:, 2] < z0)
        if below0.any():
            warnings.append(f"{int(below0.sum())} ray origins below the terrain surface")

        active = ~never & ~below0 & (t_start < t_end)
        for i in np.flatnonzero(never | below0 | ~active):
            issues[i] = TerrainIssue.TERRAIN_NO_INTERSECTION
            if not never[i] and not below0[i]:
                # the search range ends before the ray descends to the raster maximum
                ps = o[i] + t_start[i] * d[i]
                if not inside0[i] or not self.contains_xy(ps[0], ps[1]):
                    issues[i] = TerrainIssue.TERRAIN_OUTSIDE
        seen_inside = np.zeros(n, bool)
        prev_t = t_start.copy()
        k0 = 0
        hit_lo = np.full(n, np.nan)
        hit_hi = np.full(n, np.nan)
        while active.any():
            idx = np.flatnonzero(active)
            ks = np.arange(k0, k0 + STEP_BLOCK)
            t = t_start[idx, None] + dt[idx, None] * ks[None, :]
            beyond = t > t_end[idx, None]
            t = np.minimum(t, t_end[idx, None])
            px = o[idx, None, 0] + t * d[idx, None, 0]
            py = o[idx, None, 1] + t * d[idx, None, 1]
            pz = o[idx, None, 2] + t * d[idx, None, 2]
            win = g.window_for(px[~beyond], py[~beyond]) if (~beyond).any() else None
            h, inside, nd = g.sample(px, py, window=win)
            m = len(idx)
            local = np.arange(STEP_BLOCK)[None, :]
            any_beyond = beyond.any(axis=1)
            first_beyond = np.where(any_beyond, np.argmax(beyond, axis=1), STEP_BLOCK)
            in_range = local <= first_beyond[:, None]
            seen_before = np.concatenate(
                [seen_inside[idx, None], np.logical_or.accumulate(inside, axis=1)[:, :-1]],
                axis=1) | seen_inside[idx, None]
            out_ev = ~inside & seen_before
            nd_ev = inside & nd
            hit_ev = inside & ~nd & (pz - h <= 0)
            event = (out_ev | nd_ev | hit_ev) & in_range
            has = event.any(axis=1)
            k = np.where(has, np.argmax(event, axis=1), 0)
            rows = np.arange(m)
            n_eval = np.where(has, k + 1, np.minimum(first_beyond + 1, STEP_BLOCK))
            iters[idx] += n_eval
            t_prev_row = np.concatenate([prev_t[idx, None], t[:, :-1]], axis=1)
            for j in np.flatnonzero(has):
                i, kk = idx[j], k[j]
                if out_ev[j, kk]:
                    issues[i] = TerrainIssue.TERRAIN_OUTSIDE
                elif nd_ev[j, kk]:
                    issues[i], nodata_hit[i] = TerrainIssue.TERRAIN_NODATA, True
                else:
                    hit_lo[i], hit_hi[i] = t_prev_row[j, kk], t[j, kk]
            seen_inside[idx] |= (inside & in_range).any(axis=1)
            finished = ~has & any_beyond            # reached t_end without an event
            for j in np.flatnonzero(finished):
                i = idx[j]
                issues[i] = (TerrainIssue.TERRAIN_NO_INTERSECTION if seen_inside[i]
                             else TerrainIssue.TERRAIN_OUTSIDE)
            prev_t[idx] = t[rows, -1]
            active[idx[has | finished]] = False
            k0 += STEP_BLOCK

        # bisection on the bracket [lo (above), hi (at / below)]
        hit = np.flatnonzero(np.isfinite(hit_hi))
        if hit.size:
            lo, hi = hit_lo[hit].copy(), hit_hi[hit].copy()
            ok = np.zeros(hit.size, bool)
            bad = np.zeros(hit.size, bool)
            t_hit = np.full(hit.size, np.nan)
            for _ in range(MAX_BISECTION):
                mid = 0.5 * (lo + hi)
                p = o[hit] + mid[:, None] * d[hit]
                hz, ins, ndm = g.sample(p[:, 0], p[:, 1])
                bad |= (~ins | ndm) & ~ok
                diff = p[:, 2] - hz
                above = diff > 0
                lo = np.where(above & ~ok, mid, lo)
                hi = np.where(~above & ~ok, mid, hi)
                iters[hit] += (~ok).astype(int)
                conv = ~ok & ~bad & ((np.abs(diff) < self.tolerance_m) | ((hi - lo) < 1e-4))
                t_hit[conv] = mid[conv]
                ok |= conv
                if (ok | bad).all():
                    break
            for j, i in enumerate(hit):
                if bad[j]:
                    issues[i], nodata_hit[i] = TerrainIssue.TERRAIN_NODATA, True
                elif not ok[j]:
                    issues[i] = TerrainIssue.TERRAIN_NONCONVERGENT
                else:
                    xyz[i] = o[i] + t_hit[j] * d[i]
                    valid[i] = True
        return TerrainIntersectionResult(
            xyz=xyz, valid=valid, provider=self.name, method="RAY_MARCH_BISECTION",
            iterations=iters, inside_bounds=inside0, nodata_hit=nodata_hit, issues=issues,
            vertical_datum=self.vertical_datum,
            confidence=float(valid.mean()) if n else 0.0, warnings=warnings)
