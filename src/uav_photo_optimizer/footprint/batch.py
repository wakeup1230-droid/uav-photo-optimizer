"""
Batch footprint estimation with Height Strategy v2 (Phase 5.1).

AUTO, per photo:
    1. validated LRF ray height (LRF_RAY_VERTICAL, quality VALID)
    2. NEIGHBOR_LRF_INTERPOLATED — previous + next validated photo of the same flight,
       same FlightStrip and same capture group (NADIR, or OBLIQUE with the same 45° look
       sector); ground elevation interpolated linearly along the strip axis
    3. NEIGHBOR_LRF_NEAREST — only one such neighbour (lower confidence)
    4. HEIGHT_UNRESOLVED — no footprint; the photo must be protected downstream

Never across TURN photos, strips or flights. RelativeAltitude is never used here (only with
an explicit ``HeightStrategy.TAKEOFF_RELATIVE``). Thresholds come from the measured spacing
and terrain gradient of validated photos (internal research notes).
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from ..camera.intrinsics import resolve_intrinsics
from ..camera.models import LensMode
from ..flight.models import CaptureType, FlightAnalysis
from ..metadata.altitude import HeightMethod, HeightStatus, HeightStrategy
from ..metadata.models import PhotoMetadata
from .base import FootprintProjector, FootprintUnavailableError, HeightUnresolvedError, \
    ProjectionContext
from .height import neighbour_height
from .models import PhotoFootprint


@dataclass(frozen=True)
class NeighbourConfig:
    """
    Chosen from a reference dataset (internal research notes): validated same-group neighbours are
    29 m / 3.2 s apart (median; p99 51 m / 5.3 s); leave-one-out error (% of height):
    interpolation p95 10.7 % (20-30 m), 13.3 % (30-40 m), p99 jumps to 26 % beyond 40 m;
    nearest p95 15 % within 20 m, 22-34 % beyond.
    """

    max_interpolation_side_m: float = 40.0     # each side, along-track
    max_nearest_m: float = 20.0                # single-sided
    max_neighbour_time_s: float = 6.0
    look_sector_deg: float = 45.0


class HeightResolution(BaseModel):
    model_config = ConfigDict(extra="forbid")

    photo_id: str
    status: HeightStatus
    method: Optional[HeightMethod] = None
    lrf_status: Optional[str] = None
    neighbours: list[str] = Field(default_factory=list)
    neighbour_distance_m: list[float] = Field(default_factory=list)
    reason: str = ""


@dataclass
class BatchResult:
    footprints: dict[str, PhotoFootprint] = field(default_factory=dict)
    unavailable: dict[str, str] = field(default_factory=dict)      # incl. HEIGHT_UNRESOLVED
    resolutions: dict[str, HeightResolution] = field(default_factory=dict)

    @property
    def unresolved(self) -> list[str]:
        return [p for p, r in self.resolutions.items()
                if r.status is HeightStatus.HEIGHT_UNRESOLVED]


def capture_group(analysis: FlightAnalysis, photo_id: str, sector_deg: float = 45.0):
    t = analysis.photos[photo_id]
    if t.capture_type is CaptureType.OBLIQUE and t.gimbal_yaw_grid is not None:
        return ("OBLIQUE", int(round(t.gimbal_yaw_grid / sector_deg)) % int(360 / sector_deg))
    return (t.capture_type.value, None)


def project_all(photos: list[PhotoMetadata], analysis: Optional[FlightAnalysis],
                projector: FootprintProjector, context: ProjectionContext,
                lens_mode: LensMode = LensMode.AUTO,
                neighbours: NeighbourConfig = NeighbourConfig()) -> BatchResult:
    out = BatchResult()
    pending: dict[str, object] = {}
    by_id = {m.photo_id: m for m in photos}
    cams = {}
    for m in photos:
        cam = resolve_intrinsics(m, lens_mode=lens_mode)
        if cam is None:
            out.unavailable[m.photo_id] = "camera intrinsics unknown"
            continue
        cams[m.photo_id] = cam
        try:
            fp = projector.project(m, cam, context)
            out.footprints[m.photo_id] = fp
            out.resolutions[m.photo_id] = HeightResolution(
                photo_id=m.photo_id, status=HeightStatus.RESOLVED,
                method=HeightMethod(fp.height_strategy),
                lrf_status=fp.lrf_quality.status.value if fp.lrf_quality else None)
        except HeightUnresolvedError as exc:
            pending[m.photo_id] = exc
        except FootprintUnavailableError as exc:
            out.unavailable[m.photo_id] = str(exc)

    if context.height_strategy is not HeightStrategy.AUTO or not pending:
        for pid, exc in pending.items():
            _unresolved(out, pid, exc, "no neighbour interpolation for this strategy")
        return out

    # validated photos per (flight, strip, capture group), in capture order
    groups = defaultdict(list)
    if analysis is not None:
        for pid, fp in out.footprints.items():
            t = analysis.photos.get(pid)
            if t is None or t.strip_id is None or fp.height_strategy != "LRF_RAY_VERTICAL":
                continue
            groups[(t.flight_id, t.strip_id, capture_group(analysis, pid,
                                                           neighbours.look_sector_deg))].append(pid)
        for g in groups.values():
            g.sort(key=lambda p: analysis.photos[p].sequence_index)
    strips = {s.strip_id: s for s in analysis.strips} if analysis else {}

    for pid, exc in pending.items():
        t = analysis.photos.get(pid) if analysis else None
        if t is None or t.strip_id is None:
            _unresolved(out, pid, exc, "not in a FlightStrip (TURN / no flight analysis)")
            continue
        key = (t.flight_id, t.strip_id, capture_group(analysis, pid, neighbours.look_sector_deg))
        cand = groups.get(key, [])
        prev = [p for p in cand if analysis.photos[p].sequence_index < t.sequence_index]
        nxt = [p for p in cand if analysis.photos[p].sequence_index > t.sequence_index]
        heading = math.radians(strips[t.strip_id].travel_heading)
        ux, uy = math.sin(heading), math.cos(heading)

        def along(p):
            x, y = analysis.photos[p].xy
            return x * ux + y * uy

        def ok(p, limit):
            dt = abs((by_id[p].capture_time_utc or by_id[p].capture_time)
                     - (by_id[pid].capture_time_utc or by_id[pid].capture_time)).total_seconds()
            return abs(along(p) - along(pid)) <= limit and dt <= neighbours.max_neighbour_time_s

        lim_i, lim_n = neighbours.max_interpolation_side_m, neighbours.max_nearest_m
        a = prev[-1] if prev and ok(prev[-1], lim_i) else None
        b = nxt[0] if nxt and ok(nxt[0], lim_i) else None
        if not (a and b):          # single-sided: only the strict nearest limit
            a = a if a and ok(a, lim_n) else None
            b = b if b and ok(b, lim_n) else None
        ground = {p: out.footprints[p].provenance["ground_elevation"] for p in (a, b) if p}
        if a and b:
            sa, sb, s0 = along(a), along(b), along(pid)
            f = 0.0 if sb == sa else (s0 - sa) / (sb - sa)
            g = ground[a] + f * (ground[b] - ground[a])
            method, used = HeightMethod.NEIGHBOR_LRF_INTERPOLATED, [a, b]
        elif a or b:
            n = a or b
            g, method, used = ground[n], HeightMethod.NEIGHBOR_LRF_NEAREST, [n]
        else:
            _unresolved(out, pid, exc, "no validated neighbour (interpolation ≤ "
                        f"{lim_i:g} m each side, nearest ≤ {lim_n:g} m, ≤ "
                        f"{neighbours.max_neighbour_time_s:g} s) in the same strip and group")
            continue
        h = neighbour_height(by_id[pid], g, method, used)
        lrf_q = getattr(exc, "lrf_quality", None)
        try:
            fp = projector.project(by_id[pid], cams[pid], context, height=h, lrf_quality=lrf_q)
        except FootprintUnavailableError as exc2:
            _unresolved(out, pid, exc, f"neighbour height not projectable: {exc2}")
            continue
        out.footprints[pid] = fp
        out.resolutions[pid] = HeightResolution(
            photo_id=pid, status=HeightStatus.RESOLVED, method=method,
            lrf_status=lrf_q.status.value if lrf_q else None, neighbours=used,
            neighbour_distance_m=[abs(along(p) - along(pid)) for p in used])
    return out


def _unresolved(out: BatchResult, pid: str, exc, reason: str) -> None:
    lrf_q = getattr(exc, "lrf_quality", None)
    out.unavailable[pid] = f"HEIGHT_UNRESOLVED: {reason}"
    out.resolutions[pid] = HeightResolution(
        photo_id=pid, status=HeightStatus.HEIGHT_UNRESOLVED,
        lrf_status=lrf_q.status.value if lrf_q else None, reason=reason)
