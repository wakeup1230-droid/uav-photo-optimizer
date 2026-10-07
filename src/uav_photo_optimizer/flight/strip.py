"""
Flight and Flight Strip detection (Phase 4).

Pipeline
    photos ordered by capture time (UTCAtExposure → DateTimeOriginal → file sequence)
      → flights: split on time gap, sequence reset, implausible spatial jump, folder change
      → GPS projected to the metric CRS
      → local trajectory heading (central difference over ≥ min_step_m; FlightYaw only when
        the aircraft is hovering)
      → initial segments of near-constant heading
      → classify: STRIP (straight, long, parallel to the flight's main axis) / TRANSIT
        (straight, other axis) / TURN / UNCLASSIFIED
      → merge collinear compatible STRIP segments
      → endpoint refinement: extend each strip by adjacent photos lying on the strip line
        (central-difference headings of a line's first / last photo mix in the turn)
      → FlightStrip[] + per-photo PhotoTrack (capture type, relative view yaw / direction)

GimbalYaw is never used for strips — only for the oblique view direction afterwards.
All thresholds live in ``FlightConfig`` (configurable).
"""

from __future__ import annotations

import math
import re
from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from ..footprint.pose import grid_north_offset, wgs84_to
from ..metadata.models import PhotoMetadata
from .models import (CaptureType, Flight, FlightAnalysis, FlightStrip, PhotoTrack,
                     SegmentType, ViewDirection)

_SEQ_RE = re.compile(r"_(\d{3,5})(?:_[A-Z])?\.[A-Za-z]+$")


@dataclass(frozen=True)
class FlightConfig:
    # flights
    max_gap_s: float = 60.0
    max_jump_speed_mps: float = 40.0          # consecutive photos implying faster = new flight
    max_jump_m: float = 200.0                 # ... only if also farther than this
    split_on_folder: bool = True              # folder change is a hint, combined with the rest
    # headings / segments
    min_step_m: float = 3.0                   # min displacement for a trajectory heading
    heading_window: int = 4                   # neighbours searched on each side
    heading_tol_deg: float = 15.0             # photo vs running segment heading
    # strips
    min_strip_length_m: float = 80.0
    min_strip_photos: int = 8
    max_cross_track_rms_m: float = 8.0
    transit_axis_tol_deg: float = 20.0        # straight segment vs flight main axis
    merge_heading_tol_deg: float = 10.0
    merge_lateral_m: float = 15.0
    merge_max_gap_photos: int = 3
    endpoint_lateral_tol_m: float = 3.0       # endpoint refinement: max offset from line
    # capture type (GimbalPitch, degrees)
    nadir_max_pitch: float = -80.0            # pitch <= this → NADIR
    oblique_max_pitch: float = -20.0          # nadir_max < pitch <= this → OBLIQUE
    # view direction sectors (half width around 0/45/90/... degrees)
    view_sector_half_width_deg: float = 22.5


def angdiff(a: float, b: float) -> float:
    """Signed smallest difference a − b in (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def bearing(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dx, dy)) % 360.0


def circular_mean(degrees: list[float]) -> float:
    s = sum(math.sin(math.radians(d)) for d in degrees)
    c = sum(math.cos(math.radians(d)) for d in degrees)
    return math.degrees(math.atan2(s, c)) % 360.0


def axis_mean(degrees: list[float], weights: list[float] | None = None) -> float:
    """Mean of undirected axes (period 180°), result in [0, 180)."""
    w = weights or [1.0] * len(degrees)
    s = sum(wi * math.sin(math.radians(2 * d)) for d, wi in zip(degrees, w))
    c = sum(wi * math.cos(math.radians(2 * d)) for d, wi in zip(degrees, w))
    return (math.degrees(math.atan2(s, c)) / 2.0) % 180.0


def capture_type(pitch: float | None, cfg: FlightConfig) -> CaptureType:
    if pitch is None:
        return CaptureType.OTHER
    if pitch <= cfg.nadir_max_pitch:
        return CaptureType.NADIR
    if pitch <= cfg.oblique_max_pitch:
        return CaptureType.OBLIQUE
    return CaptureType.OTHER


_SECTORS = [ViewDirection.FORWARD, ViewDirection.FORWARD_RIGHT, ViewDirection.RIGHT,
            ViewDirection.BACKWARD_RIGHT, ViewDirection.BACKWARD, ViewDirection.BACKWARD_LEFT,
            ViewDirection.LEFT, ViewDirection.FORWARD_LEFT]


def view_direction(relative_yaw: float, cfg: FlightConfig) -> ViewDirection:
    for i, sector in enumerate(_SECTORS):
        if abs(angdiff(relative_yaw, i * 45.0)) <= cfg.view_sector_half_width_deg:
            return sector
    return ViewDirection.OTHER


def sequence_number(meta: PhotoMetadata) -> int | None:
    m = _SEQ_RE.search(meta.filename)
    return int(m.group(1)) if m else None


def _time(meta: PhotoMetadata):
    return meta.capture_time_utc or meta.capture_time


class StripDetector(ABC):
    @abstractmethod
    def analyse(self, photos: list[PhotoMetadata], crs: str) -> FlightAnalysis:
        ...


class TrajectoryStripDetector(StripDetector):
    def __init__(self, config: FlightConfig = FlightConfig()):
        self.cfg = config

    # -- flights -------------------------------------------------------------------------

    def detect_flights(self, photos: list[PhotoMetadata], xy: dict[str, tuple[float, float]]
                       ) -> list[tuple[Flight, list[PhotoMetadata]]]:
        cfg = self.cfg
        ordered = sorted((p for p in photos if p.has_gps and _time(p) is not None),
                         key=lambda p: (_time(p), sequence_number(p) or 0, p.photo_id))
        flights, current, reason = [], [], "first photo"
        for p in ordered:
            if current:
                q = current[-1]
                dt = (_time(p) - _time(q)).total_seconds()
                dist = math.dist(xy[p.photo_id], xy[q.photo_id])
                sp, sq = sequence_number(p), sequence_number(q)
                why = ""
                if dt > cfg.max_gap_s:
                    why = f"time gap {dt:.0f} s"
                elif sp is not None and sq is not None and sp < sq:
                    why = f"sequence reset {sq}→{sp}"
                elif dist > cfg.max_jump_m and dist / max(dt, 0.5) > cfg.max_jump_speed_mps:
                    why = f"spatial jump {dist:.0f} m in {dt:.1f} s"
                elif cfg.split_on_folder and _folder(p) != _folder(q):
                    why = "folder change"
                if why:
                    flights.append((reason, current))
                    current, reason = [], why
            current.append(p)
        if current:
            flights.append((reason, current))
        out = []
        for i, (why, ms) in enumerate(flights, start=1):
            out.append((Flight(flight_id=f"F{i:02d}", photo_ids=[m.photo_id for m in ms],
                               start_time=_time(ms[0]), end_time=_time(ms[-1]),
                               folders=sorted({_folder(m) for m in ms}), split_reason=why), ms))
        return out

    # -- headings ------------------------------------------------------------------------

    def _headings(self, ms: list[PhotoMetadata], pts: np.ndarray, grid_offsets: list[float]
                  ) -> list[tuple[float | None, str]]:
        cfg, n = self.cfg, len(ms)
        out = []
        for i in range(n):
            prev = next((j for j in range(i - 1, max(-1, i - 1 - cfg.heading_window), -1)
                         if np.linalg.norm(pts[i] - pts[j]) >= cfg.min_step_m), None)
            nxt = next((j for j in range(i + 1, min(n, i + 1 + cfg.heading_window))
                        if np.linalg.norm(pts[j] - pts[i]) >= cfg.min_step_m), None)
            a = pts[prev] if prev is not None else pts[i]
            b = pts[nxt] if nxt is not None else pts[i]
            if np.linalg.norm(b - a) >= cfg.min_step_m:
                out.append((bearing(*(b - a)), "TRAJECTORY"))
            elif ms[i].flight_yaw is not None:
                out.append(((ms[i].flight_yaw + grid_offsets[i]) % 360.0, "FLIGHT_YAW"))
            else:
                out.append((None, ""))
        return out

    # -- segments ------------------------------------------------------------------------

    def _segments(self, headings: list[float | None]) -> list[list[int]]:
        segs, cur, ref = [], [], None
        for i, h in enumerate(headings):
            if h is None:
                if cur:
                    segs.append(cur)
                segs.append([i])
                cur, ref = [], None
                continue
            if cur and abs(angdiff(h, ref)) <= self.cfg.heading_tol_deg:
                cur.append(i)
                ref = circular_mean([headings[j] for j in cur])
            else:
                if cur:
                    segs.append(cur)
                cur, ref = [i], h
        if cur:
            segs.append(cur)
        return segs

    @staticmethod
    def _line(pts: np.ndarray):
        """PCA line: centre, unit axis, along-track coords, cross-track RMS."""
        c = pts.mean(axis=0)
        if len(pts) < 2:
            return c, np.array([0.0, 1.0]), np.zeros(len(pts)), 0.0
        u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
        axis = vt[0]
        along = (pts - c) @ axis
        cross = (pts - c) @ np.array([-axis[1], axis[0]])
        return c, axis, along, float(np.sqrt(np.mean(cross ** 2)))

    def _is_straight(self, pts: np.ndarray) -> bool:
        if len(pts) < self.cfg.min_strip_photos:
            return False
        _, _, along, rms = self._line(pts)
        return (along.max() - along.min()) >= self.cfg.min_strip_length_m and \
            rms <= self.cfg.max_cross_track_rms_m

    # -- main ------------------------------------------------------------------------------

    def analyse(self, photos: list[PhotoMetadata], crs: str) -> FlightAnalysis:
        cfg = self.cfg
        to_grid = wgs84_to(crs)
        xy = {p.photo_id: to_grid.transform(p.longitude, p.latitude)
              for p in photos if p.has_gps}
        result = FlightAnalysis(crs=crs)
        skipped = [p.photo_id for p in photos if not p.has_gps or _time(p) is None]
        if skipped:
            result.warnings.append(f"{len(skipped)} photos without GPS or time not analysed")

        strip_no = 0
        for flight, ms in self.detect_flights(photos, xy):
            result.flights.append(flight)
            pts = np.array([xy[m.photo_id] for m in ms])
            offsets = [grid_north_offset(m.longitude, m.latitude, crs) for m in ms]
            heads = self._headings(ms, pts, offsets)
            segs = self._segments([h for h, _ in heads])

            straight = [self._is_straight(pts[s]) for s in segs]
            lengths = []
            axes = []
            for s, ok in zip(segs, straight):
                if ok:
                    _, ax, along, _ = self._line(pts[s])
                    axes.append(bearing(*ax) % 180.0)
                    lengths.append(float(along.max() - along.min()))
            main_axis = axis_mean(axes, lengths) if axes else None

            seg_types = []
            for s, ok in zip(segs, straight):
                if ok:
                    _, ax, _, _ = self._line(pts[s])
                    parallel = main_axis is not None and \
                        abs(angdiff(2 * (bearing(*ax) % 180.0), 2 * main_axis)) / 2 \
                        <= cfg.transit_axis_tol_deg
                    seg_types.append(SegmentType.STRIP if parallel else SegmentType.TRANSIT)
                elif len(s) == 1 and heads[s[0]][0] is None:
                    seg_types.append(SegmentType.UNCLASSIFIED)
                else:
                    seg_types.append(SegmentType.TURN)

            # merge collinear compatible STRIP segments separated by <= N short non-strip photos
            groups: list[list[int]] = []   # lists of segment indices forming one strip
            for k, t in enumerate(seg_types):
                if t is not SegmentType.STRIP:
                    continue
                if groups:
                    last = groups[-1][-1]
                    between = sum(len(segs[j]) for j in range(last + 1, k))
                    if between <= cfg.merge_max_gap_photos and \
                            self._compatible(pts, heads, segs[groups[-1][0]:last + 1], segs[k]):
                        groups[-1].append(k)
                        continue
                groups.append([k])

            seg_of_photo = {}
            for k, s in enumerate(segs):
                for i in s:
                    seg_of_photo[i] = k
            photo_strip: dict[int, str] = {}
            group_idx = [sorted(i for k in range(g[0], g[-1] + 1) for i in segs[k])
                         for g in groups]
            for gi, idx in enumerate(group_idx):
                for i in idx:
                    photo_strip[i] = f"G{gi}"
            yaw_grid = [((m.flight_yaw + offsets[i]) % 360.0) if m.flight_yaw is not None
                        else None for i, m in enumerate(ms)]
            for gi, idx in enumerate(group_idx):
                group_idx[gi] = self._extend(idx, pts, photo_strip, f"G{gi}", yaw_grid)
            for g, idx in zip(groups, group_idx):
                strip_no += 1
                sid = f"S{strip_no:03d}"
                for i in idx:
                    photo_strip[i] = sid
                spts = pts[idx]
                c, ax, along, rms = self._line(spts)
                travel = bearing(*(spts[-1] - spts[0]))
                axis_b = bearing(*ax)
                if abs(angdiff(axis_b, travel)) > 90:
                    axis_b = (axis_b + 180) % 360
                caps = [capture_type(ms[i].gimbal_pitch, cfg) for i in idx]
                result.strips.append(FlightStrip(
                    strip_id=sid, flight_id=flight.flight_id,
                    photo_ids=[ms[i].photo_id for i in idx],
                    heading=axis_b, travel_heading=axis_b, axis_heading=axis_b % 180.0,
                    length_m=float(along.max() - along.min()),
                    start_xy=tuple(spts[0]), end_xy=tuple(spts[-1]), centre_xy=tuple(c),
                    cross_track_rms_m=rms, n_nadir=caps.count(CaptureType.NADIR),
                    n_oblique=caps.count(CaptureType.OBLIQUE), merged_from=len(g)))
            strip_by_id = {s.strip_id: s for s in result.strips}

            for i, m in enumerate(ms):
                k = seg_of_photo[i]
                sid = photo_strip.get(i)
                st = strip_by_id.get(sid)
                seg_type = SegmentType.STRIP if st is not None else (
                    SegmentType.TURN if seg_types[k] is SegmentType.STRIP else seg_types[k])
                cap = capture_type(m.gimbal_pitch, cfg)
                gy = (m.gimbal_yaw + offsets[i]) % 360.0 if m.gimbal_yaw is not None else None
                rel, view = None, ViewDirection.NOT_APPLICABLE
                if st is not None and cap is CaptureType.OBLIQUE and gy is not None:
                    rel = angdiff(gy, st.travel_heading)
                    view = view_direction(rel, cfg)
                result.photos[m.photo_id] = PhotoTrack(
                    photo_id=m.photo_id, flight_id=flight.flight_id, sequence_index=i,
                    xy=tuple(pts[i]), segment_type=seg_type, segment_index=k,
                    strip_id=sid, capture_type=cap, track_heading=heads[i][0],
                    heading_source=heads[i][1],
                    travel_heading=st.travel_heading if st else None,
                    axis_heading=st.axis_heading if st else None,
                    gimbal_pitch=m.gimbal_pitch, gimbal_yaw_grid=gy,
                    relative_view_yaw=rel, view_direction=view)
        return result

    def _extend(self, idx: list[int], pts: np.ndarray, owner: dict[int, str],
                me: str, yaw_grid: list) -> list[int]:
        """
        Grow a strip at both ends with sequence-adjacent photos lying on its line and flown
        in the strip direction (FlightYaw within heading_tol) — photos taken while the
        aircraft already rotates for the turn stay TURN.
        """
        c, ax, along, _ = self._line(pts[idx])
        normal = np.array([-ax[1], ax[0]])
        travel_sign = 1.0 if (pts[idx[-1]] - pts[idx[0]]) @ ax >= 0 else -1.0
        travel = bearing(*(travel_sign * ax))
        idx = list(idx)

        def on_line(j, beyond_end: bool) -> bool:
            if owner.get(j, me) != me:
                return False
            off = abs((pts[j] - c) @ normal)
            pos = travel_sign * ((pts[j] - c) @ ax)
            lim = travel_sign * ((pts[idx[-1] if beyond_end else idx[0]] - c) @ ax)
            ahead = pos >= lim if beyond_end else pos <= lim
            yaw_ok = yaw_grid[j] is None or \
                abs(angdiff(yaw_grid[j], travel)) <= self.cfg.heading_tol_deg
            return off <= self.cfg.endpoint_lateral_tol_m and ahead and yaw_ok

        j = idx[0] - 1
        while j >= 0 and on_line(j, beyond_end=False):
            idx.insert(0, j)
            j -= 1
        j = idx[-1] + 1
        while j < len(pts) and on_line(j, beyond_end=True):
            idx.append(j)
            j += 1
        return idx

    def _compatible(self, pts, heads, prev_segs, seg) -> bool:
        a = [i for s in prev_segs for i in s]
        ha = circular_mean([heads[i][0] for i in a if heads[i][0] is not None])
        hb = circular_mean([heads[i][0] for i in seg if heads[i][0] is not None])
        if abs(angdiff(ha, hb)) > self.cfg.merge_heading_tol_deg:
            return False
        c, ax, _, _ = self._line(pts[a])
        normal = np.array([-ax[1], ax[0]])
        lateral = np.abs((pts[seg] - c) @ normal).max()
        return lateral <= self.cfg.merge_lateral_m


def _folder(meta: PhotoMetadata) -> str:
    parts = meta.photo_id.split("/")
    return parts[0] if len(parts) > 1 else ""


def analyse_flights(photos: list[PhotoMetadata], crs: str,
                    config: FlightConfig = FlightConfig()) -> FlightAnalysis:
    return TrajectoryStripDetector(config).analyse(photos, crs)
