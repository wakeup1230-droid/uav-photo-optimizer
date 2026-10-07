"""
Phase 6A Photo Selection Optimizer — dry run (Experimental).

Fixed order:
    1. analyse baseline (candidates = footprint ∩ coverage area; HEIGHT_UNRESOLVED photos whose
       GPS point lies in the area)
    2. find baseline defects (front / side / oblique along / oblique cross gaps, coverage gaps,
       low-confidence geometry)
    3. mark protected photos (defect zones, unresolved / low-confidence heights, turns,
       non-standard capture directions, thin AOI-boundary coverage)
    4. nadir along-track: per strip segment, minimum-photo path through legal edges
       (front overlap of the two kept photos ≥ target), must-keep photos enforced (DP)
    5. nadir whole-strip evaluation (cross-track): removable only if the two neighbours
       overlap ≥ side target
    6. oblique along-track: same DP per strip and look group (groups never mixed);
       oblique cross-track is evaluated only
    7.-9. validation (coverage per capture group, new front / side / along / cross defects,
       worsened baseline defects, boundary multiplicity) with restore passes
    10. SelectionPlan; status INVALID unless every safety counter is 0

Original gaps are never bridged: a consecutive pair below target splits the DP segment.
Nothing is copied or deleted.
"""

from __future__ import annotations

import math
from collections import defaultdict
from datetime import datetime
from typing import Optional

import numpy as np
import shapely
from shapely.geometry import mapping
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from ..core.config import MIN_OVERLAP
from ..flight.models import CaptureType
from ..overlap.engine import OverlapConfig, OverlapEngine, unit
from ..overlap.front import along_track_overlap
from ..overlap.side import cross_track_overlap
from .base import PhotoSelector
from .models import (BaselineDefect, CoverageSummary, DecisionStatus, DefectSeverity, DefectType,
                     PhotoDecision, PlanStatus, SafetyReport, SelectionOptimizerConfig,
                     SelectionPlan, SelectionReason, StripEvaluation)

EPS_PCT = 1e-3            # float tolerance on overlap comparisons (percentage points)
R = SelectionReason

MUST_KEEP_PRIORITY = [    # strongest reason wins when several apply
    R.PROTECTED_HEIGHT_UNRESOLVED, R.PROTECTED_LOW_CONFIDENCE_HEIGHT, R.PROTECTED_TURN,
    R.PROTECTED_BASELINE_DEFECT, R.KEEP_CAPTURE_DIRECTION, R.KEEP_AOI_BOUNDARY,
    R.KEEP_STRIP_ENDPOINT,
]
PROTECTED_REASONS = {R.PROTECTED_HEIGHT_UNRESOLVED, R.PROTECTED_LOW_CONFIDENCE_HEIGHT,
                     R.PROTECTED_TURN, R.PROTECTED_BASELINE_DEFECT}


def _severity(value_pct: float, target: float) -> DefectSeverity:
    if value_pct < MIN_OVERLAP:
        return DefectSeverity.HIGH
    return DefectSeverity.MEDIUM if value_pct < target - 5 else DefectSeverity.LOW


class DPSelectionOptimizer(PhotoSelector):
    """Graph / dynamic-programming optimizer (no OR-Tools)."""

    # -- setup -----------------------------------------------------------------------------

    def plan(self, geometry, area: BaseGeometry,
             config: SelectionOptimizerConfig = SelectionOptimizerConfig()) -> SelectionPlan:
        self.cfg, self.area = config, area
        self.fl, self.fps = geometry.flights, geometry.footprints
        self.strips = {s.strip_id: s for s in self.fl.strips}
        res = geometry.height_resolutions
        shapely.prepare(area)

        # 1. candidates
        cand = []
        for pid, t in self.fl.photos.items():
            fp = self.fps.get(pid)
            if fp is not None:
                if shapely.intersects(area, fp.geometry):
                    cand.append(pid)
            elif shapely.contains_xy(area, *t.xy):
                cand.append(pid)                  # unresolved / no footprint: GPS point
        self.cand = sorted(cand, key=lambda p: (self.fl.photos[p].flight_id,
                                                self.fl.photos[p].sequence_index))
        self.cset = set(self.cand)
        self.group = {p: self._group(p) for p in self.cand}
        self.seq = defaultdict(list)                       # (strip, group) → ordered pids
        for p in self.cand:
            t = self.fl.photos[p]
            if t.strip_id:
                self.seq[(t.strip_id, self.group[p])].append(p)
        self.low_conf = {p for p in self.cand
                         if p not in self.fps or self.fps[p].height_confidence
                         < config.low_confidence_height - 1e-9}
        self.unresolved = {p for p in self.cand if p not in self.fps}

        warnings = ["Plan only: nothing is copied or deleted by the optimizer."]
        must: dict[str, set] = defaultdict(set)
        # 2. defects
        defects = self._baseline_defects()
        # 3. protection
        self._protect(defects, must)
        removed: dict[str, SelectionReason] = {}
        kept_reason: dict[str, SelectionReason] = {}
        strip_evals: list[StripEvaluation] = []
        # 4. nadir along-track
        if config.optimize_nadir_along:
            self._optimise_along("NADIR", config.front_target_pct, must, removed, kept_reason,
                                 R.REMOVE_REDUNDANT_ALONG_TRACK, R.KEEP_REQUIRED_FRONT_OVERLAP)
        # 5. nadir strip evaluation
        if config.evaluate_nadir_strips:
            strip_evals = self._evaluate_strips(must, removed)
        # 6. oblique along-track (each look group separately)
        if config.optimize_oblique_along:
            self._optimise_along("OBLIQUE", config.front_target_pct, must, removed,
                                 kept_reason, R.REMOVE_REDUNDANT_OBLIQUE_ALONG_TRACK,
                                 R.KEEP_REQUIRED_ALONG_TRACK_OVERLAP)
        # 7.-9. validation + restore
        restored: dict[str, str] = {}
        safety = SafetyReport()
        for _ in range(config.max_restore_iterations):
            kept = self.cset - set(removed)
            safety, fixes = self._validate(kept, removed, defects)
            if safety.total == 0:
                break
            if not fixes:
                safety.details.append("violations remain but no removed photo can fix them")
                break
            for pid, why in fixes.items():
                removed.pop(pid, None)
                restored[pid] = why
        else:
            warnings.append("restore pass stopped at max_restore_iterations")
        kept = self.cset - set(removed)
        if safety.total:
            safety, _ = self._validate(kept, removed, defects)

        # 10. plan
        decisions = []
        for p in self.cand:
            t = self.fl.photos[p]
            reasons = must.get(p, set())
            strongest = next((r for r in MUST_KEEP_PRIORITY if r in reasons), None)
            detail = {}
            if p in restored:
                status, reason, detail = DecisionStatus.KEEP, R.RESTORED_BY_VALIDATION, \
                    {"restored_by": restored[p]}
            elif p in removed:
                status, reason = DecisionStatus.REMOVE, removed[p]
            elif strongest in PROTECTED_REASONS:
                status, reason = DecisionStatus.PROTECTED, strongest
            elif strongest is not None:
                status, reason = DecisionStatus.KEEP, strongest
            else:
                status, reason = DecisionStatus.KEEP, kept_reason.get(p, R.KEEP_NOT_OPTIMIZED)
            if reasons:
                detail["all_reasons"] = sorted(r.value for r in reasons)
            decisions.append(PhotoDecision(
                photo_id=p, status=status, reason=reason, capture_type=t.capture_type.value,
                view_group=self.group[p], flight_id=t.flight_id, strip_id=t.strip_id,
                detail=detail))

        counts = defaultdict(lambda: defaultdict(int))
        by_reason = defaultdict(int)
        for d in decisions:
            counts[d.capture_type][d.status.value] += 1
            by_reason[d.reason.value] += 1
        n_remove = sum(d.status is DecisionStatus.REMOVE for d in decisions)
        n_prot = sum(d.status is DecisionStatus.PROTECTED for d in decisions)
        ok = safety.total == 0
        self.final_removed, self.final_restored = dict(removed), dict(restored)
        self.defects = defects
        return SelectionPlan(
            status=PlanStatus.VALID if ok else PlanStatus.INVALID, config=config,
            input_photo_count=geometry.photos_scanned, candidate_photo_count=len(self.cand),
            keep_count=len(decisions) - n_remove - n_prot, remove_count=n_remove,
            protected_count=n_prot,
            reduction_percent=round(100.0 * n_remove / max(1, len(self.cand)), 3),
            counts_by_capture={k: dict(v) for k, v in counts.items()},
            counts_by_reason=dict(by_reason), restored_count=len(restored),
            decisions=decisions, baseline_defects=defects,
            coverage_before=self._coverage_summary(self.cset),
            coverage_after=self._coverage_summary(kept),
            strip_evaluations=strip_evals, safety=safety, constraints_passed=ok,
            geometry_valid=ok, exportable=ok, warnings=warnings, created_at=datetime.now())

    # -- helpers -----------------------------------------------------------------------------

    def _group(self, pid: str) -> str:
        t = self.fl.photos[pid]
        if t.capture_type is CaptureType.NADIR:
            return "NADIR"
        if t.capture_type is CaptureType.OBLIQUE and t.gimbal_yaw_grid is not None:
            return f"OBLIQUE_S{int(round(t.gimbal_yaw_grid / 45.0)) % 8}"
        return "OTHER"

    def _heading(self, pid: str) -> float:
        return self.strips[self.fl.photos[pid].strip_id].travel_heading

    def _along(self, a: str, b: str) -> Optional[float]:
        if a not in self.fps or b not in self.fps:
            return None
        return 100 * along_track_overlap(self.fps[a].geometry, self.fps[b].geometry,
                                         self._heading(a)).over_max

    def _cross(self, a: str, b: str) -> Optional[float]:
        if a not in self.fps or b not in self.fps:
            return None
        return 100 * cross_track_overlap(self.fps[a].geometry, self.fps[b].geometry,
                                         self._heading(a)).over_max

    def _evidence(self, p: str) -> bool:
        """May this photo's footprint be used as overlap evidence?"""
        return p in self.fps and p not in self.low_conf

    # -- side / cross partners -----------------------------------------------------------------

    def _strip_pairs(self, cap: CaptureType, ids: set):
        """Neighbour strips (by geometry) among the strips that still have ``cap`` photos."""
        eng = OverlapEngine(OverlapConfig())
        sub = self.fl.model_copy(update={"photos": {p: t for p, t in self.fl.photos.items()
                                                    if p in ids}})
        strips = [s.model_copy(update={"photo_ids": [p for p in s.photo_ids if p in ids]})
                  for s in self.fl.strips]
        return eng.pair_strips([s for s in strips if s.photo_ids], sub, cap)

    def _pairs_for(self, kept: set, cap: CaptureType):
        live = frozenset(s.strip_id for s in self.fl.strips
                         if any(p in kept and self.fl.photos[p].capture_type is cap
                                for p in s.photo_ids))
        if not hasattr(self, "_pairs_cache"):
            self._pairs_cache = {}
        key = (cap, live)
        if key not in self._pairs_cache:
            self._pairs_cache[key] = self._strip_pairs(cap, kept)
        if not hasattr(self, "base_pairs"):
            self.base_pairs = {}
        self.base_pairs.setdefault(cap, self._pairs_cache[key])
        return self._pairs_cache[key]

    def _partners(self, kept: set, cap: CaptureType) -> dict:
        """(a, side) → (b, value %, strip_b): nearest kept partner of kept photo a in the
        neighbouring strip on that side (same look group); pairing recomputed when whole
        strips are removed."""
        out = {}
        for sp in self._pairs_for(kept, cap):
            u, _ = unit(self.strips[sp.strip_a].travel_heading)
            a_ids = [p for p in self.strips[sp.strip_a].photo_ids
                     if p in kept and self.fl.photos[p].capture_type is cap and p in self.fps]
            b_all = [p for p in self.strips[sp.strip_b].photo_ids
                     if p in kept and self.fl.photos[p].capture_type is cap and p in self.fps]
            for a in a_ids:
                b_ids = [b for b in b_all if self.group[b] == self.group[a]]
                if not b_ids:
                    out[(a, sp.side)] = (None, 0.0, sp.strip_b)
                    continue
                pos = np.dot(self.fl.photos[a].xy, u)
                b = min(b_ids, key=lambda q: abs(np.dot(self.fl.photos[q].xy, u) - pos))
                pts = np.asarray(self.fps[a].geometry.exterior.coords) @ u
                if abs(np.dot(self.fl.photos[b].xy, u) - pos) > 0.5 * (pts.max() - pts.min()):
                    out[(a, sp.side)] = (None, 0.0, sp.strip_b)
                    continue
                out[(a, sp.side)] = (b, self._cross(a, b), sp.strip_b)
        return out

    # -- 2. baseline defects ------------------------------------------------------------------

    def _baseline_defects(self) -> list[BaselineDefect]:
        cfg, defects = self.cfg, []
        self.defect_pairs = set()            # along-track pairs below target (baseline)
        self.base_along = {}                 # (a, b) consecutive → value
        for (sid, grp), ps in self.seq.items():
            if grp == "OTHER":
                continue
            kind = DefectType.FRONT_GAP if grp == "NADIR" else DefectType.ALONG_TRACK_GAP
            for a, b in zip(ps, ps[1:]):
                v = self._along(a, b)
                self.base_along[(a, b)] = v
                if v is None or v + EPS_PCT < cfg.front_target_pct:
                    self.defect_pairs.add((a, b))
                    if v is None:
                        continue                   # unresolved → LOW_CONFIDENCE defect below
                    defects.append(BaselineDefect(
                        defect_id=f"D{len(defects) + 1:04d}", type=kind,
                        flight_id=self.fl.photos[a].flight_id, strip_id=sid, view_group=grp,
                        affected_photos=[a, b], baseline_metric=round(v, 3),
                        target_metric=cfg.front_target_pct,
                        severity=_severity(v, cfg.front_target_pct)))
        self.base_side = {}
        for cap, kind in ((CaptureType.NADIR, DefectType.SIDE_GAP),
                          (CaptureType.OBLIQUE, DefectType.CROSS_TRACK_GAP)):
            for (a, side), (b, v, sb) in self._partners(self.cset, cap).items():
                self.base_side[(a, side)] = (b, v)
                if b is not None and v + EPS_PCT < cfg.side_target_pct:
                    defects.append(BaselineDefect(
                        defect_id=f"D{len(defects) + 1:04d}", type=kind,
                        flight_id=self.fl.photos[a].flight_id,
                        strip_id=self.fl.photos[a].strip_id, strip_id_b=sb,
                        view_group=self.group[a], affected_photos=[a, b],
                        baseline_metric=round(v, 3), target_metric=cfg.side_target_pct,
                        severity=_severity(v, cfg.side_target_pct)))
        # coverage gaps: coverage area not seen by any candidate footprint
        self.base_cov = self._coverage(self.cset)
        gap = self.area.difference(unary_union(list(self.base_cov.values())))
        for g in getattr(gap, "geoms", [gap]):
            if not g.is_empty and g.area > cfg.coverage_tolerance_m2:
                defects.append(BaselineDefect(
                    defect_id=f"D{len(defects) + 1:04d}", type=DefectType.AOI_COVERAGE_GAP,
                    affected_geometry=mapping(g), baseline_metric=round(g.area, 2),
                    target_metric=0.0, severity=DefectSeverity.HIGH,
                    detail="coverage area not covered by any candidate footprint"))
        for p in sorted(self.low_conf):
            t = self.fl.photos[p]
            defects.append(BaselineDefect(
                defect_id=f"D{len(defects) + 1:04d}", type=DefectType.LOW_CONFIDENCE_GEOMETRY,
                flight_id=t.flight_id, strip_id=t.strip_id, view_group=self.group[p],
                affected_photos=[p], severity=DefectSeverity.MEDIUM,
                detail="HEIGHT_UNRESOLVED" if p in self.unresolved else
                f"height_confidence {self.fps[p].height_confidence}"))
        return defects

    # -- 3. protection --------------------------------------------------------------------------

    def _protect(self, defects: list[BaselineDefect], must: dict) -> None:
        k = self.cfg.defect_protection_neighbours
        index = {}
        for key, ps in self.seq.items():
            for i, p in enumerate(ps):
                index[p] = (key, i)

        def zone(p):
            if p not in index:
                return [p]
            key, i = index[p]
            ps = self.seq[key]
            return ps[max(0, i - k): i + k + 1]

        for d in defects:
            if d.type in (DefectType.AOI_COVERAGE_GAP, DefectType.LOW_CONFIDENCE_GEOMETRY):
                continue
            for p in d.affected_photos:
                for q in zone(p):
                    must[q].add(R.PROTECTED_BASELINE_DEFECT)
        for p in self.cand:
            t = self.fl.photos[p]
            if p in self.unresolved:
                must[p].add(R.PROTECTED_HEIGHT_UNRESOLVED)
            elif p in self.low_conf:
                must[p].add(R.PROTECTED_LOW_CONFIDENCE_HEIGHT)
            if t.strip_id is None:
                must[p].add(R.PROTECTED_TURN)
            if self.group[p] == "OTHER":
                must[p].add(R.KEEP_CAPTURE_DIRECTION)
        for key, ps in self.seq.items():
            if ps:
                must[ps[0]].add(R.KEEP_STRIP_ENDPOINT)
                must[ps[-1]].add(R.KEEP_STRIP_ENDPOINT)
        # thin coverage near the coverage-area border
        self.band_pts = self._band_points()
        self.base_mult = self._multiplicity(self.cset)
        kmin = self.cfg.boundary_min_multiplicity
        for grp, (counts, covering) in self.base_mult.items():
            for i in np.nonzero((counts > 0) & (counts <= kmin))[0]:
                for p in covering[i]:
                    must[p].add(R.KEEP_AOI_BOUNDARY)

    def _band_points(self) -> np.ndarray:
        g = self.cfg.boundary_guard_m
        if g <= 0:
            return np.empty((0, 2))
        band = self.area.difference(self.area.buffer(-g))
        minx, miny, maxx, maxy = band.bounds
        s = self.cfg.boundary_sample_spacing_m
        xs, ys = np.meshgrid(np.arange(minx, maxx, s), np.arange(miny, maxy, s))
        pts = np.column_stack([xs.ravel(), ys.ravel()])
        return pts[shapely.contains_xy(band, pts[:, 0], pts[:, 1])]

    def _multiplicity(self, ids: set) -> dict:
        """group → (count per band point, covering photo list per point)."""
        out = {}
        groups = defaultdict(list)
        for p in sorted(ids):                         # deterministic order (hash-seed safe)
            if p in self.fps:
                groups[self.group[p]].append(p)
        for grp, ps in groups.items():
            counts = np.zeros(len(self.band_pts), dtype=int)
            covering = [[] for _ in range(len(self.band_pts))]
            for p in ps:
                geom = self.fps[p].geometry
                minx, miny, maxx, maxy = geom.bounds
                bb = np.nonzero((self.band_pts[:, 0] >= minx) & (self.band_pts[:, 0] <= maxx)
                                & (self.band_pts[:, 1] >= miny) & (self.band_pts[:, 1] <= maxy))[0]
                if len(bb) == 0:
                    continue
                inside = bb[shapely.contains_xy(geom, self.band_pts[bb, 0], self.band_pts[bb, 1])]
                counts[inside] += 1
                for i in inside:
                    covering[i].append(p)
            out[grp] = (counts, covering)
        return out

    # -- 4./6. along-track DP -------------------------------------------------------------------

    def _optimise_along(self, cap: str, target: float, must: dict, removed: dict,
                        kept_reason: dict, remove_reason: SelectionReason,
                        keep_reason: SelectionReason) -> None:
        for (sid, grp), ps in self.seq.items():
            if grp == "OTHER" or (cap == "NADIR") != (grp == "NADIR"):
                continue
            # segments: split at baseline gaps and at photos that cannot serve as evidence
            segments, cur = [], []
            for i, p in enumerate(ps):
                if not self._evidence(p):
                    if cur:
                        segments.append(cur)
                    cur = []
                    continue
                if cur and (cur[-1], p) in self.defect_pairs:
                    segments.append(cur)
                    cur = []
                cur.append(p)
            if cur:
                segments.append(cur)
            for seg in segments:
                keep = self._dp(seg, target, must)
                for p in seg:
                    if p in keep:
                        kept_reason.setdefault(p, keep_reason)
                    elif not must.get(p):
                        removed[p] = remove_reason

    def _dp(self, seg: list[str], target: float, must: dict) -> set:
        """Minimum-photo path from seg[0] to seg[-1]; must-keep photos are mandatory."""
        n = len(seg)
        if n <= 2:
            return set(seg)
        forced = [i for i, p in enumerate(seg) if must.get(p)]
        forced = sorted({0, n - 1, *forced})
        keep = set()
        for lo, hi in zip(forced, forced[1:]):
            best = {lo: (0, -math.inf, None)}         # node → (count, -min_overlap, prev)
            for j in range(lo + 1, hi + 1):
                cand = None
                for i in range(j - 1, lo - 1, -1):
                    if i not in best:
                        continue
                    v = self._along(seg[i], seg[j])
                    if v is None or v + EPS_PCT < target:
                        if j - i > 1:
                            continue
                        break                         # consecutive pair below target: stop
                    c, negmin, _ = best[i]
                    score = (c + 1, max(negmin, -v))
                    if cand is None or score < cand[:2]:
                        cand = (*score, i)
                if cand is not None:
                    best[j] = cand
            if hi not in best:                        # no legal path → keep everything
                keep.update(seg[lo:hi + 1])
                continue
            j = hi
            while j is not None:
                keep.add(seg[j])
                j = best[j][2]
        return keep

    # -- 5. whole-strip evaluation --------------------------------------------------------------

    def _evaluate_strips(self, must: dict, removed: dict) -> list[StripEvaluation]:
        pairs = self.base_pairs[CaptureType.NADIR]
        by_strip = defaultdict(dict)
        for sp in pairs:
            by_strip[sp.strip_a][sp.side] = sp.strip_b
        evals = []
        for sid in sorted(by_strip):
            nadir = [p for p in self.seq.get((sid, "NADIR"), [])]
            if not nadir:
                continue
            left, right = by_strip[sid].get("LEFT"), by_strip[sid].get("RIGHT")
            ev = StripEvaluation(strip_id=sid, left_strip=left, right_strip=right)
            if not (left and right):
                ev.reason = "outer strip (no neighbour on one side) — kept"
                evals.append(ev)
                continue
            vals = []
            u, _ = unit(self.strips[left].travel_heading)
            rights = [q for q in self.seq.get((right, "NADIR"), []) if q in self.fps]
            for a in self.seq.get((left, "NADIR"), []):
                if a not in self.fps or not rights:
                    continue
                pos = np.dot(self.fl.photos[a].xy, u)
                b = min(rights, key=lambda q: abs(np.dot(self.fl.photos[q].xy, u) - pos))
                vals.append(self._cross(a, b))
            ev.neighbours_side_overlap_min_pct = round(min(vals), 3) if vals else None
            protected = any(must.get(p, set()) - {R.KEEP_STRIP_ENDPOINT} for p in nadir)
            if not vals or min(vals) + EPS_PCT < self.cfg.side_target_pct:
                ev.reason = (f"neighbours {left}↔{right} side overlap "
                             f"{min(vals):.1f} % < target — kept" if vals else
                             "no neighbour pairs — kept")
            elif protected:
                ev.removable, ev.reason = True, "removable by overlap, but contains protected photos — kept"
            else:
                ev.removable = ev.removed = True
                ev.reason = "neighbours overlap ≥ side target — strip removed"
                for p in nadir:
                    removed[p] = R.REMOVE_REDUNDANT_CROSS_TRACK
            evals.append(ev)
        return evals

    # -- 7.-9. validation ---------------------------------------------------------------------

    def _coverage(self, ids: set) -> dict:
        groups = defaultdict(list)
        for p in sorted(ids):
            if p in self.fps:
                groups[self.group[p]].append(self.fps[p].geometry)
        return {g: unary_union(geoms).intersection(self.area) for g, geoms in groups.items()}

    def _coverage_summary(self, ids: set) -> CoverageSummary:
        cov = self._coverage(ids)
        total = unary_union(list(cov.values())).area if cov else 0.0
        return CoverageSummary(area_m2=round(self.area.area, 2), covered_m2=round(total, 2),
                               covered_ratio=round(total / self.area.area, 6),
                               by_group_m2={g: round(c.area, 2) for g, c in sorted(cov.items())})

    def _validate(self, kept: set, removed: dict, defects) -> tuple[SafetyReport, dict]:
        cfg = self.cfg
        s, fixes = SafetyReport(), {}
        rem = set(removed)

        def restore(p, why):
            if p in rem and p not in fixes:
                fixes[p] = why

        # coverage per capture group
        cov = self._coverage(kept)
        for grp, base in self.base_cov.items():
            lost = base.difference(cov.get(grp, base.difference(base)))
            if lost.area > cfg.coverage_tolerance_m2:
                s.new_coverage_holes += 1
                s.new_coverage_hole_m2 += lost.area
                s.details.append(f"coverage hole {grp}: {lost.area:.1f} m²")
                best = sorted((p for p in sorted(rem)
                               if self.group.get(p) == grp and p in self.fps),
                              key=lambda p: -self.fps[p].geometry.intersection(lost).area)
                if best and self.fps[best[0]].geometry.intersection(lost).area > 0:
                    restore(best[0], f"coverage hole {grp}")
        # along-track (front / oblique along)
        for (sid, grp), ps in self.seq.items():
            if grp == "OTHER":
                continue
            ks = [p for p in ps if p in kept]
            for a, b in zip(ks, ks[1:]):
                if not (self._evidence(a) and self._evidence(b)):
                    continue
                v = self._along(a, b)
                base = self.base_along.get((a, b))
                if (a, b) in self.base_along and (base is None or base + EPS_PCT < cfg.front_target_pct):
                    if base is not None and v + EPS_PCT < base:
                        s.baseline_defects_worsened += 1
                    continue                          # unchanged baseline defect pair
                if v + EPS_PCT < cfg.front_target_pct:
                    if grp == "NADIR":
                        s.new_front_defects += 1
                    else:
                        s.new_along_track_defects += 1
                    i, j = ps.index(a), ps.index(b)
                    between = [p for p in ps[i + 1:j] if p in rem]
                    if between:
                        restore(between[len(between) // 2], f"along-track {grp} {a}->{b}")
        # cross-track (nadir side / oblique cross)
        for cap in (CaptureType.NADIR, CaptureType.OBLIQUE):
            now = self._partners(kept, cap)
            for (a, side), (b, v, sb) in now.items():
                base_b, base_v = self.base_side.get((a, side), (None, 0.0))
                if base_b is None:
                    continue
                if base_v + EPS_PCT < cfg.side_target_pct:          # baseline defect
                    if b is None or v + EPS_PCT < base_v:
                        s.baseline_defects_worsened += 1
                        restore(base_b, f"worsened side defect {a}")
                    continue
                if b is None or v + EPS_PCT < cfg.side_target_pct:
                    if cap is CaptureType.NADIR:
                        s.new_side_defects += 1
                    else:
                        s.new_cross_track_defects += 1
                    restore(base_b, f"cross-track {cap.value} {a}")
        # boundary multiplicity
        kmin = cfg.boundary_min_multiplicity
        mult = self._multiplicity(kept)
        for grp, (base_counts, covering) in self.base_mult.items():
            counts = mult.get(grp, (np.zeros_like(base_counts), None))[0]
            need = np.minimum(base_counts, kmin)
            bad = np.nonzero(counts < need)[0]
            if len(bad):
                s.boundary_multiplicity_violations += len(bad)
                s.details.append(f"boundary multiplicity {grp}: {len(bad)} points")
                for i in bad:
                    cand = [p for p in covering[i] if p in rem]
                    if cand:
                        restore(cand[0], f"boundary multiplicity {grp}")
        s.new_coverage_hole_m2 = round(s.new_coverage_hole_m2, 3)
        return s, fixes


def plan_selection(geometry, area: BaseGeometry,
                   config: SelectionOptimizerConfig = SelectionOptimizerConfig()) -> SelectionPlan:
    return DPSelectionOptimizer().plan(geometry, area, config)
