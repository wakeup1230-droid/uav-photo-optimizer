"""
Overlap Engine (Phase 5) — computes directional and shared-coverage overlap; removes nothing.

Inputs: Estimated Ground Footprints (``PhotoFootprint``) + ``FlightAnalysis`` (Phase 4).

Strip frame: along-track unit vector u = (sin θ, cos θ) for strip travel heading θ (grid),
cross-track unit vector v = (cos θ, −sin θ) (pointing to the right of travel).
Axis overlap: project every footprint vertex on the axis → interval; 1-D intersection.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
from shapely.geometry.base import BaseGeometry

from ..core.config import DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP
from ..flight.models import CaptureType, FlightAnalysis, FlightStrip
from ..flight.strip import angdiff
from ..footprint.models import PhotoFootprint
from .models import (AxisOverlap, OverlapKind, OverlapMethod, OverlapReport, OverlapResult,
                     PhotoAdjacencyGraph, SharedCoverage, StripPair, TerrainMode)

TERRAIN_WARNING = "TERRAIN_NOT_ACCOUNTED_FOR"
PASS_EPS_PCT = 1e-3        # float tolerance for the PASS/FAIL comparison (percentage points)


@dataclass(frozen=True)
class OverlapConfig:
    front_target_pct: float = DEFAULT_FRONT_OVERLAP
    side_target_pct: float = DEFAULT_SIDE_OVERLAP
    max_neighbour_rank: int = 2                  # along-track pairs: i+1 (rank 1) and i+2
    pair_axis_tol_deg: float = 10.0              # strips must be parallel
    pair_min_along_overlap: float = 0.2          # shared along-track extent / shorter strip
    pair_min_cross_distance_m: float = 5.0       # closer = same line, not a neighbour
    side_max_along_offset_ratio: float = 0.5     # partner within 0.5 × footprint length
    look_sector_deg: float = 45.0


def unit(heading: float) -> tuple[np.ndarray, np.ndarray]:
    t = math.radians(heading)
    return np.array([math.sin(t), math.cos(t)]), np.array([math.cos(t), -math.sin(t)])


def interval(geom: BaseGeometry, axis: np.ndarray) -> tuple[float, float]:
    pts = np.asarray(geom.exterior.coords)
    proj = pts @ axis
    return float(proj.min()), float(proj.max())


def axis_overlap(a: BaseGeometry, b: BaseGeometry, axis: np.ndarray, heading: float,
                 name: str) -> AxisOverlap:
    a0, a1 = interval(a, axis)
    b0, b1 = interval(b, axis)
    la, lb = a1 - a0, b1 - b0
    ov = max(0.0, min(a1, b1) - max(a0, b0))
    return AxisOverlap(axis=name, axis_heading=heading % 360.0, length_a=la, length_b=lb,
                       overlap_length=ov, over_min=ov / min(la, lb), over_a=ov / la,
                       over_b=ov / lb, over_max=ov / max(la, lb))


def shared_coverage(a: BaseGeometry, b: BaseGeometry) -> SharedCoverage:
    inter = a.intersection(b).area
    union = a.area + b.area - inter
    return SharedCoverage(area_a=a.area, area_b=b.area, intersection_area=inter,
                          over_a=inter / a.area, over_b=inter / b.area,
                          over_min_area=inter / min(a.area, b.area),
                          iou=inter / union if union > 0 else 0.0)


def summary(values: Iterable[float]) -> dict:
    a = np.asarray(list(values), dtype=float)
    if a.size == 0:
        return {"n": 0}
    q = np.percentile(a, [0, 5, 25, 50, 75, 95, 100])
    return {"n": int(a.size), "min": float(q[0]), "p05": float(q[1]), "p25": float(q[2]),
            "median": float(q[3]), "p75": float(q[4]), "p95": float(q[5]), "max": float(q[6]),
            "mean": float(a.mean())}


class OverlapEngine:
    def __init__(self, config: OverlapConfig = OverlapConfig()):
        self.cfg = config

    # -- strip pairing ---------------------------------------------------------------------

    def pair_strips(self, strips: list[FlightStrip], analysis: FlightAnalysis,
                    capture: Optional[CaptureType] = None) -> list[StripPair]:
        """Nearest parallel strip on each side (LEFT / RIGHT) with along-track overlap."""
        cfg = self.cfg

        def pts(s):
            ids = [p for p in s.photo_ids
                   if capture is None or analysis.photos[p].capture_type is capture]
            return np.array([analysis.photos[p].xy for p in ids]) if ids else None

        cache = {s.strip_id: pts(s) for s in strips}
        pairs = []
        for a in strips:
            pa = cache[a.strip_id]
            if pa is None or len(pa) < 2:
                continue
            u, v = unit(a.travel_heading)
            ca = pa.mean(axis=0)
            a0, a1 = (pa @ u).min(), (pa @ u).max()
            best = {}
            for b in strips:
                pb = cache[b.strip_id]
                if b is a or pb is None or len(pb) < 2:
                    continue
                d_axis = abs(angdiff(2 * a.axis_heading, 2 * b.axis_heading)) / 2
                if d_axis > cfg.pair_axis_tol_deg:
                    continue
                b0, b1 = (pb @ u).min(), (pb @ u).max()
                shared = max(0.0, min(a1, b1) - max(a0, b0))
                ratio = shared / max(1e-9, min(a1 - a0, b1 - b0))
                if ratio < cfg.pair_min_along_overlap:
                    continue
                cross = float((pb.mean(axis=0) - ca) @ v)
                if abs(cross) < cfg.pair_min_cross_distance_m:
                    continue
                side = "RIGHT" if cross > 0 else "LEFT"
                if side not in best or abs(cross) < abs(best[side][1]):
                    best[side] = (b, cross, d_axis, ratio)
            for side, (b, cross, d_axis, ratio) in best.items():
                pairs.append(StripPair(strip_a=a.strip_id, strip_b=b.strip_id, side=side,
                                       axis_difference_deg=d_axis,
                                       cross_track_distance_m=abs(cross),
                                       along_track_overlap_ratio=ratio,
                                       same_flight=a.flight_id == b.flight_id))
        return pairs

    # -- pair result -----------------------------------------------------------------------

    def _result(self, kind: OverlapKind, method: OverlapMethod, fa: PhotoFootprint,
                fb: PhotoFootprint, frame_heading: float, analysis: FlightAnalysis,
                times: dict, rank: int = 1, sector: Optional[int] = None) -> OverlapResult:
        u, v = unit(frame_heading)
        along = kind in (OverlapKind.FRONT_OVERLAP, OverlapKind.ALONG_TRACK_GEOMETRIC_OVERLAP)
        ax = axis_overlap(fa.geometry, fb.geometry, u if along else v,
                          frame_heading if along else frame_heading + 90.0,
                          "ALONG" if along else "CROSS")
        pa, pb = analysis.photos[fa.photo_id], analysis.photos[fb.photo_id]
        target = self.cfg.front_target_pct if along else self.cfg.side_target_pct
        warnings = sorted({TERRAIN_WARNING, *(w.value for w in fa.warnings),
                           *(w.value for w in fb.warnings)})
        ta, tb = times.get(fa.photo_id), times.get(fb.photo_id)
        hs = fa.height_strategy if fa.height_strategy == fb.height_strategy else \
            f"{fa.height_strategy}+{fb.height_strategy}"
        return OverlapResult(
            photo_a=fa.photo_id, photo_b=fb.photo_id, kind=kind, value=ax.over_max, axis=ax,
            shared=shared_coverage(fa.geometry, fb.geometry),
            strip_a=pa.strip_id, strip_b=pb.strip_id, same_strip=pa.strip_id == pb.strip_id,
            neighbour_rank=rank, look_sector=sector,
            distance_m=math.dist(fa.camera_xy, fb.camera_xy),
            capture_time_delta_s=(tb - ta).total_seconds() if ta and tb else None,
            geometry_method=method,
            terrain_mode=(TerrainMode.TERRAIN_PROVIDER if fa.method.value == fb.method.value
                          == "TERRAIN_RASTER" else TerrainMode.LOCAL_HORIZONTAL_PLANE),
            height_strategy=hs, confidence=min(fa.confidence, fb.confidence),
            warnings=warnings, target_pct=target, target_pass=ax.over_max * 100
            >= target - PASS_EPS_PCT)

    # -- main ------------------------------------------------------------------------------

    def compute(self, footprints: dict[str, PhotoFootprint], analysis: FlightAnalysis,
                times: Optional[dict] = None) -> OverlapReport:
        cfg = self.cfg
        times = times or {}
        report = OverlapReport(crs=analysis.crs, front_target_pct=cfg.front_target_pct,
                               side_target_pct=cfg.side_target_pct)
        edges: list[OverlapResult] = []
        strips = {s.strip_id: s for s in analysis.strips}

        def sector(pid) -> Optional[int]:
            g = analysis.photos[pid].gimbal_yaw_grid
            return None if g is None else int(round(g / cfg.look_sector_deg)) % 8

        def members(s: FlightStrip, cap: CaptureType) -> list[str]:
            return [p for p in s.photo_ids if p in footprints
                    and analysis.photos[p].capture_type is cap]

        # along-track: nadir FRONT and oblique ALONG (same absolute look sector)
        for s in analysis.strips:
            nadir = members(s, CaptureType.NADIR)
            for r in range(1, cfg.max_neighbour_rank + 1):
                for a, b in zip(nadir, nadir[r:]):
                    edges.append(self._result(OverlapKind.FRONT_OVERLAP,
                                              OverlapMethod.NADIR_GEOMETRIC_PLANAR,
                                              footprints[a], footprints[b], s.travel_heading,
                                              analysis, times, rank=r))
            by_sector = defaultdict(list)
            for p in members(s, CaptureType.OBLIQUE):
                by_sector[sector(p)].append(p)
            for sec, ps in by_sector.items():
                for r in range(1, cfg.max_neighbour_rank + 1):
                    for a, b in zip(ps, ps[r:]):
                        edges.append(self._result(OverlapKind.ALONG_TRACK_GEOMETRIC_OVERLAP,
                                                  OverlapMethod.OBLIQUE_GEOMETRIC_PLANAR,
                                                  footprints[a], footprints[b],
                                                  s.travel_heading, analysis, times, rank=r,
                                                  sector=sec))

        # cross-track: neighbouring strips
        nadir_pairs = self.pair_strips(analysis.strips, analysis, CaptureType.NADIR)
        obl_pairs = self.pair_strips(analysis.strips, analysis, CaptureType.OBLIQUE)
        report.strip_pairs = nadir_pairs
        seen = set()
        for pairs, cap in ((nadir_pairs, CaptureType.NADIR), (obl_pairs, CaptureType.OBLIQUE)):
            for sp in pairs:
                key = (cap, *sorted((sp.strip_a, sp.strip_b)))
                if key in seen:
                    continue
                seen.add(key)
                a_s, b_s = strips[sp.strip_a], strips[sp.strip_b]
                u, _ = unit(a_s.travel_heading)
                a_ids, b_ids = members(a_s, cap), members(b_s, cap)
                if not a_ids or not b_ids:
                    continue
                b_along = np.array([np.asarray(analysis.photos[p].xy) @ u for p in b_ids])
                for pa in a_ids:
                    fa = footprints[pa]
                    pos = np.asarray(analysis.photos[pa].xy) @ u
                    cand = range(len(b_ids))
                    if cap is CaptureType.OBLIQUE:
                        cand = [i for i in cand if sector(b_ids[i]) == sector(pa)]
                    if not cand:
                        continue
                    i = min(cand, key=lambda i: abs(b_along[i] - pos))
                    fb = footprints[b_ids[i]]
                    length = interval(fa.geometry, u)
                    if abs(b_along[i] - pos) > cfg.side_max_along_offset_ratio * \
                            (length[1] - length[0]):
                        continue
                    kind = (OverlapKind.SIDE_OVERLAP if cap is CaptureType.NADIR
                            else OverlapKind.CROSS_TRACK_GEOMETRIC_OVERLAP)
                    method = (OverlapMethod.NADIR_GEOMETRIC_PLANAR if cap is CaptureType.NADIR
                              else OverlapMethod.OBLIQUE_GEOMETRIC_PLANAR)
                    edges.append(self._result(kind, method, fa, fb, a_s.travel_heading,
                                              analysis, times,
                                              sector=sector(pa) if cap is CaptureType.OBLIQUE
                                              else None))

        report.graph = PhotoAdjacencyGraph(nodes=sorted(footprints), edges=edges)
        report.warnings.append("All overlaps use per-photo planar Estimated Ground Footprints "
                               "(TERRAIN_NOT_ACCOUNTED_FOR); oblique values are geometric, not "
                               "photogrammetric overlap.")
        return report


def overlap_statistics(report: OverlapReport, analysis: FlightAnalysis) -> dict:
    """C11 statistics: per kind (rank 1), plus by flight / strip."""
    out = {}
    for kind in OverlapKind:
        res = report.results(kind)
        vals = [r.value * 100 for r in res]
        by_flight, by_strip = defaultdict(list), defaultdict(list)
        for r in res:
            by_flight[analysis.photos[r.photo_a].flight_id].append(r.value * 100)
            by_strip[r.strip_a if r.same_strip else f"{r.strip_a}|{r.strip_b}"].append(
                r.value * 100)
        target = report.front_target_pct if kind in (
            OverlapKind.FRONT_OVERLAP, OverlapKind.ALONG_TRACK_GEOMETRIC_OVERLAP) \
            else report.side_target_pct
        out[kind.value] = {
            "pct": summary(vals),
            "target_pct": target,
            "pass_rate": float(np.mean([r.target_pass for r in res])) if res else None,
            "shared_iou_pct": summary(r.shared.iou * 100 for r in res),
            "by_flight": {k: summary(v) for k, v in sorted(by_flight.items())},
            "by_strip": {k: summary(v) for k, v in sorted(by_strip.items())},
        }
    return out
